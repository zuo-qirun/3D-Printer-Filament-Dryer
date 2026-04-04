#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SHT31.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "dryer_common.h"
#include "pid_autotune.h"
#include "NebulaDeckMenu.h"

/*
接线图（ESP32-S3）：

I2C：
  GPIO8  (SDA) -> AHT10 SDA + OLED SDA
  GPIO9  (SCL) -> AHT10 SCL + OLED SCL
  3V3          -> AHT10 VCC
  GND          -> AHT10 GND + OLED GND
  OLED I2C 地址：0x78（8 位写地址，等价 7 位地址 0x3C）

加热（12V PTC，N-MOS 低边开关）：
  GPIO4  -> MOSFET Gate（串联约 100R），Gate 下拉约 10k 到 GND
  12V+   -> PTC+
  PTC-   -> MOSFET Drain
  MOSFET Source -> GND
  ESP32 GND 与 12V GND 必须共地

四线风扇：
  12V+   -> Fan V+
  GND    -> Fan GND（与 ESP32 共地）
  GPIO5  -> Fan PWM 输入（建议按风扇规格使用开漏/电平转换）
  Fan TACH -> （当前固件未启用测速输入）

五键按键（默认低电平按下，使用 INPUT_PULLUP）：
  上键 -> GPIO10
  下键 -> GPIO11
  左键 -> GPIO12
  右键 -> GPIO13
  中键 -> GPIO14
*/
constexpr uint8_t I2C_SDA_PIN = 8;        // I2C SDA：连接 AHT10/OLED 数据线。
constexpr uint8_t I2C_SCL_PIN = 9;        // I2C SCL：连接 AHT10/OLED 时钟线。
constexpr uint8_t OLED_ADDR_8BIT = 0x78;  // OLED 8 位 I2C 地址（对应 7 位 0x3C）。
constexpr uint8_t HEATER_PIN = 4;         // 加热 MOSFET 栅极控制 GPIO。
constexpr uint8_t FAN_PIN = 5;            // 四线风扇 PWM 控速 GPIO。

constexpr uint8_t BTN_UP_PIN = 10;     // 上键 GPIO（菜单上移/数值增加）。
constexpr uint8_t BTN_DOWN_PIN = 11;   // 下键 GPIO（菜单下移/数值减小）。
constexpr uint8_t BTN_LEFT_PIN = 12;   // 左键 GPIO（返回/大步减小）。
constexpr uint8_t BTN_RIGHT_PIN = 13;  // 右键 GPIO（进入/大步增加）。
constexpr uint8_t BTN_OK_PIN = 14;     // 中键 GPIO（确认/进入菜单）。

constexpr uint8_t FAN_PWM_CHANNEL = 0;          // ESP32 LEDC 通道号（风扇专用）。
constexpr uint8_t FAN_PWM_RESOLUTION_BITS = 8;  // PWM 分辨率位数（0~255）。
constexpr uint32_t FAN_PWM_FREQ_HZ = 25000;     // 四线风扇建议 PWM 频率（25kHz）。
constexpr bool FAN_PWM_ACTIVE_LOW_DEFAULT = true;  // 四线风扇标准 PWM 为有效低电平，默认开启反相。
constexpr float FAN_SIGNAL_HIGH_V = 3.3f;       // ESP32 GPIO PWM 高电平电压（V）。
constexpr uint16_t FAN_EST_MAX_RPM = 7000;      // 100% PWM 对应的风扇满速估算值（可按实测修正）。

constexpr uint32_t SENSOR_INTERVAL_MS = 500;       // 传感器采样周期（ms）。
constexpr uint32_t CONTROL_INTERVAL_MS = 1000;     // PID/风扇控制周期（ms）。
constexpr uint32_t DISPLAY_INTERVAL_MS = 80;       // OLED 刷新节拍（ms）。
constexpr uint32_t PAGE_INTERVAL_MS = 3000;        // 主页温湿度自动轮播周期（ms）。
constexpr uint32_t CONTROL_WINDOW_MS = 2000;       // 时间比例加热控制窗口（ms）。
constexpr uint32_t STATE_SAVE_INTERVAL_MS = 10000; // NVS 自动保存周期（ms）。
constexpr uint32_t LOG_INTERVAL_MS = 5000;         // CSV 与串口日志输出周期（ms）。
constexpr uint16_t BUTTON_DEBOUNCE_MS = 30;        // 按键去抖时间（ms）。
constexpr float SMOOTH_ALPHA = 0.30f;              // 温湿度一阶滤波系数（越大响应越快）。
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;  // STA 模式联网超时时间（ms）。
constexpr uint16_t DNS_PORT = 53;                    // 配网页 DNS 劫持端口。
constexpr const char* WIFI_AP_SSID = "Dryer-Config"; // 设备配网热点名称。
constexpr const char* WIFI_AP_PASS = "12345678";     // 设备配网热点密码（至少 8 位）。
constexpr const char* BEMFA_HOST = "bemfa.com";
constexpr uint16_t BEMFA_MQTT_PORT = 9501;
constexpr uint16_t BEMFA_DEVICE_TYPE = 1;            // Bemfa MQTT 设备协议类型。
constexpr uint16_t MQTT_BUFFER_SIZE = 2048;
constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 8000;
constexpr uint32_t MQTT_PUBLISH_INTERVAL_MS = 5000;
constexpr uint32_t OTA_AUTO_CHECK_INTERVAL_MS = 6UL * 3600UL * 1000UL;
constexpr uint32_t HEAT_TIMEOUT_MS = 45UL * 60UL * 1000UL;
constexpr float HEAT_TIMEOUT_MARGIN_C = 5.0f;
constexpr uint32_t HEATER_STUCK_CONFIRM_MS = 30UL * 1000UL;
constexpr float HEATER_STUCK_OVER_TARGET_C = 12.0f;
constexpr uint32_t HUMIDITY_SAMPLE_INTERVAL_MS = 30000;
constexpr uint32_t HUMIDITY_STABLE_WINDOW_MS = 20UL * 60UL * 1000UL;
constexpr float HUMIDITY_STABLE_DELTA_PCT = 0.30f;
constexpr uint8_t MAX_USER_PRESETS = 8;
constexpr uint8_t HUMIDITY_HISTORY_CAPACITY = 48;
constexpr uint32_t DRYER_FIRMWARE_VERSION = 1;
constexpr float PID_DEFAULT_KP = 0.09f;              // PID 默认 Kp。
constexpr float PID_DEFAULT_KI = 0.008f;             // PID 默认 Ki。
constexpr float PID_DEFAULT_KD = 0.15f;              // PID 默认 Kd。

const PidAutoTuneConfig AUTOTUNE_CFG = [] {
  PidAutoTuneConfig c;
  c.bandC = 1.0f;                      // 继电振荡带宽（目标温度上下波动范围）。
  c.minTimeMs = 3UL * 60UL * 1000UL;   // 最小校准时长，防止样本不足。
  c.timeoutMs = 20UL * 60UL * 1000UL;  // 校准超时阈值，防止长时间卡住。
  c.relayAmplitude = 0.5f;             // 继电输出幅值（0~1 输出对应 0.5）。
  c.minAmplitudeC = 0.02f;             // 最小有效振幅（摄氏度）。
  return c;
}();

constexpr float ABS_OVER_TEMP_C = 130.0f;     // 绝对超温保护阈值（摄氏度）。
constexpr uint8_t MAX_SENSOR_FAIL_COUNT = 5;  // 连续失败上限，超过即判定传感器故障。

constexpr float TARGET_TEMP_MIN_C = 35.0f;      // 菜单可设置的最低目标温度（摄氏度）。
constexpr float TARGET_TEMP_MAX_C = 120.0f;     // 菜单可设置的最高目标温度（摄氏度）。
constexpr float IDLE_TEMP_DISABLED_C = 0.0f;    // 闲时温度关闭值（0 表示不保温）。
constexpr uint32_t DURATION_MIN_SEC = 30UL * 60UL;   // 最短烘干时长（秒）。
constexpr uint32_t DURATION_MAX_SEC = 24UL * 3600UL; // 最长烘干时长（秒）。

struct MaterialPreset {
  const char* name;
  float targetTempC;
  uint32_t durationSec;
  uint8_t fanBasePct;
  uint8_t fanMaxPct;
};

constexpr MaterialPreset PRESETS[] = {
    // 说明：以下温度/时长基于 Bambu Lab 耗材指南“鼓风型烤箱干燥条件”整理。
    {"PLA", 55.0f, 8UL * 3600UL, 30, 80},
    {"PETG_HF", 65.0f, 8UL * 3600UL, 40, 90},
    {"ABS", 80.0f, 8UL * 3600UL, 45, 95},
    {"ABS_GF", 80.0f, 8UL * 3600UL, 45, 95},
    {"ASA", 80.0f, 8UL * 3600UL, 45, 95},
    {"PC", 80.0f, 8UL * 3600UL, 50, 100},
    {"TPU95A_HF", 70.0f, 8UL * 3600UL, 35, 85},
    {"PLA_CF", 55.0f, 8UL * 3600UL, 35, 85},
    {"PETG_CF", 65.0f, 8UL * 3600UL, 40, 90},
    {"PET_CF", 80.0f, 10UL * 3600UL, 45, 95},
    {"PAHT_CF", 80.0f, 10UL * 3600UL, 50, 100},
    {"PA6_CF", 80.0f, 10UL * 3600UL, 50, 100},
    {"PA6_GF", 80.0f, 10UL * 3600UL, 50, 100},
    {"PPA_CF", 100.0f, 10UL * 3600UL, 55, 100},
    {"PPS_CF", 100.0f, 10UL * 3600UL, 55, 100},
};
constexpr size_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);
constexpr uint8_t MAIN_MENU_COUNT = 10;
constexpr const char* MAIN_MENU_ITEMS[MAIN_MENU_COUNT] = {
    "启动/停止", "目标温度", "烘干时长", "闲时温度", "空闲风扇",
    "自定义参数", "材料预设", "PID参数", "PID自校准", "清除故障",
};

enum PresetGroup : uint8_t {
  GROUP_BASIC = 0,      // 常规通用材料
  GROUP_ENGINEERING,    // 工程塑料
  GROUP_COMPOSITE,      // 纤维增强复合材料
  GROUP_COUNT,
};

constexpr uint8_t PRESET_GROUPS[PRESET_COUNT] = {
    GROUP_BASIC, GROUP_BASIC, GROUP_BASIC, GROUP_BASIC, GROUP_BASIC, GROUP_ENGINEERING,
    GROUP_BASIC, GROUP_COMPOSITE, GROUP_COMPOSITE, GROUP_COMPOSITE, GROUP_COMPOSITE,
    GROUP_COMPOSITE, GROUP_COMPOSITE, GROUP_COMPOSITE, GROUP_COMPOSITE,
};

constexpr const char* PRESET_GROUP_NAMES[GROUP_COUNT] = {
    "基础材料", "工程材料", "复合增强",
};

enum FaultFlags : uint32_t {
  FAULT_NONE = 0,
  FAULT_SENSOR = 1 << 0,
  FAULT_OVER_TEMP = 1 << 1,
  FAULT_HEAT_TIMEOUT = 1 << 2,
  FAULT_HEATER_STUCK = 1 << 3,
};

enum SensorType : uint8_t {
  SENSOR_TYPE_AHT10 = 0,
  SENSOR_TYPE_SHT3X = 1,
};

enum UiMode : uint8_t {
  UI_HOME = 0,
  UI_MENU,
  UI_SET_TEMP,
  UI_SET_TIME,
  UI_SET_IDLE_TEMP,
  UI_SET_IDLE_FAN,
  UI_SET_PRESET,
  UI_SET_PID,
};

enum ButtonId : uint8_t {
  BTN_UP = 0,
  BTN_DOWN,
  BTN_LEFT,
  BTN_RIGHT,
  BTN_OK,
  BTN_COUNT,
};

struct ButtonState {
  uint8_t pin;
  bool stableLevel;
  bool lastRead;
  uint32_t lastChangeMs;
};

struct ValueSlideAnim {
  char oldText[24];
  char newText[24];
  int8_t dir;
  float progress;
  bool active;
};

struct NumberRailAnim {
  float oldValue;
  float newValue;
  float progress;
  bool active;
};

struct UserPreset {
  bool used = false;
  char id[24] = "";
  char name[24] = "";
  float targetTempC = 50.0f;
  uint32_t durationSec = 4UL * 3600UL;
  uint8_t fanBasePct = 30;
  uint8_t fanMaxPct = 80;
  float idleTempC = IDLE_TEMP_DISABLED_C;
  uint8_t idleFanPct = 0;
  float humidityStopPct = 0.0f;
};

struct HistorySummary {
  uint32_t rows = 0;
  float minTempC = 0.0f;
  float maxTempC = 0.0f;
  float avgTempC = 0.0f;
  float minHumiPct = 0.0f;
  float maxHumiPct = 0.0f;
  float avgHumiPct = 0.0f;
  float avgHeaterPct = 0.0f;
  float avgFanPct = 0.0f;
  uint32_t activeRows = 0;
};

Adafruit_AHTX0 aht10;
Adafruit_SHT31 sht3x = Adafruit_SHT31();
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;
WebServer g_webServer(80);
DNSServer g_dnsServer;
WiFiClient g_mqttNetClient;
PubSubClient g_mqttClient(g_mqttNetClient);
uint8_t g_customFanBasePct = 30;
uint8_t g_customFanMaxPct = 80;
float g_humidityStopPct = 0.0f;
float g_humidityStableDeltaPct = HUMIDITY_STABLE_DELTA_PCT;
String g_activeCustomPresetId;
String g_activeCustomPresetName;
String g_webToken;
String g_bemfaUid;
String g_bemfaMqttKey;
String g_bemfaControlTopic = "dryer010";
String g_bemfaStatusTopic = "dryer004";
String g_bemfaOtaTopic;
bool g_bemfaNotificationsEnabled = false;
String g_bemfaNotifyGroup = "default";
bool g_mqttEnabled = false;
bool g_mqttConnected = false;
bool g_lastMqttEnabledDebug = false;
bool g_lastMqttConnectedDebug = false;
int8_t g_lastMqttClientStateDebug = 127;
uint32_t g_lastMqttReconnectMs = 0;
uint32_t g_lastMqttPublishMs = 0;
bool g_statusPublishDirty = true;
char g_lastNotifyMsg[96] = "";
char g_lastNotifyKind[16] = "none";
char g_lastOtaMsg[48] = "IDLE";
String g_lastOtaUrl;
String g_lastOtaTag;
uint32_t g_lastOtaVersion = DRYER_FIRMWARE_VERSION;
uint32_t g_lastOtaCheckMs = 0;
bool g_otaAutoCheckEnabled = false;
uint32_t g_dryingStartMs = 0;
uint32_t g_lastReachedSetpointMs = 0;
uint32_t g_heaterDemandZeroSinceMs = 0;
float g_humidityHistoryValues[HUMIDITY_HISTORY_CAPACITY] = {0};
uint32_t g_humidityHistoryTimes[HUMIDITY_HISTORY_CAPACITY] = {0};
uint8_t g_humidityHistoryCount = 0;
uint8_t g_humidityHistoryHead = 0;
uint32_t g_lastHumiditySampleMs = 0;
UserPreset g_userPresets[MAX_USER_PRESETS];
uint8_t g_userPresetCount = 0;

// 五个按键的去抖与边沿状态缓存。
ButtonState g_buttons[BTN_COUNT] = {
    {BTN_UP_PIN, HIGH, HIGH, 0},     {BTN_DOWN_PIN, HIGH, HIGH, 0},
    {BTN_LEFT_PIN, HIGH, HIGH, 0},   {BTN_RIGHT_PIN, HIGH, HIGH, 0},
    {BTN_OK_PIN, HIGH, HIGH, 0},
};

size_t g_presetIndex = 0;                // 当前材料预设索引（PRESETS 下标）。
bool g_dryingActive = false;             // 当前是否处于烘干运行状态。
uint32_t g_remainingSec = 0;             // 烘干剩余时间（秒）。
uint32_t g_faultFlags = FAULT_NONE;      // 当前故障位标志（可按位组合）。

bool g_sensorOk = false;                 // 温湿度传感器是否可用。
uint8_t g_sensorFailCount = 0;           // 传感器连续读取失败计数。
SensorType g_sensorType = SENSOR_TYPE_AHT10; // 当前温湿度传感器类型。
float g_tempC = 0.0f;                    // 原始温度读数（摄氏度）。
float g_humi = 0.0f;                     // 原始湿度读数（%RH）。
float g_smoothTempC = 0.0f;              // 平滑后的温度（用于控制与显示）。
float g_smoothHumi = 0.0f;               // 平滑后的湿度（用于显示与记录）。
bool g_smoothInited = false;             // 平滑滤波是否完成初始化。

float g_pidIntegral = 0.0f;              // PID 积分项内部状态。
float g_pidPrevTemp = 0.0f;              // PID 上次温度（用于微分项）。
bool g_pidInited = false;                // PID 内部状态初始化标志。
float g_heaterDemand = 0.0f;             // 加热需求比例（0.0~1.0）。
bool g_heaterOn = false;                 // 当前加热输出通断状态。
uint8_t g_fanPct = 0;                    // 当前风扇占空比百分比。
uint8_t g_idleFanPct = 0;                // 空闲状态风扇转速（0~100%）。
uint16_t g_fanRpm = 0;                       // 当前按 PWM 输出比例估算得到的转速（RPM）。
bool g_fanPwmActiveLow = FAN_PWM_ACTIVE_LOW_DEFAULT; // PWM 极性：true=有效低电平（反相）。
uint8_t g_fanPwmDutyRaw = 0;                  // 实际写入 LEDC 的原始 duty（0~255）。

float g_targetTempC = 50.0f;             // 用户设定目标温度（摄氏度）。
float g_idleTempC = IDLE_TEMP_DISABLED_C; // 闲时目标温度（0=关闭闲时保温）。
uint32_t g_configDurationSec = 4UL * 3600UL;  // 用户设定烘干时长（秒）。
float g_pidKp = PID_DEFAULT_KP;          // PID 比例参数 Kp。
float g_pidKi = PID_DEFAULT_KI;          // PID 积分参数 Ki。
float g_pidKd = PID_DEFAULT_KD;          // PID 微分参数 Kd。

bool g_pidAutoTuneActive = false;        // PID 自动校准是否正在运行。
PidAutoTuneState g_pidAutoTuneState;     // PID 自动校准状态机上下文。
char g_pidAutoTuneMsg[24] = "IDLE";      // 自动校准状态文本（IDLE/RUN/DONE/FAIL）。

UiMode g_uiMode = UI_HOME;               // 当前 UI 页面状态机模式。
uint8_t g_menuIndex = 0;                 // 菜单模式下的当前选中项索引。
uint8_t g_pidEditIndex = 0;              // PID 参数编辑光标（0:Kp 1:Ki 2:Kd）。
bool g_showTempPage = true;              // 主页轮播标志（true 温度页 / false 湿度页）。
uint8_t g_presetGroupFilter = GROUP_BASIC;  // 预设菜单当前分组过滤器。
bool g_userCustomMode = false;           // 参数模式：true=用户自定义，false=材料预设。
String g_wifiSsid;                       // 已保存的 Wi-Fi SSID。
String g_wifiPass;                       // 已保存的 Wi-Fi 密码。
bool g_wifiConnected = false;            // 当前是否已连接到路由器（STA）。
bool g_apConfigMode = false;             // 当前是否处于 AP 配网模式。
NebulaDeckMenu g_mainMenuFx;             // OLED 菜单动画渲染器。
ValueSlideAnim g_animSetTemp = {"", "", 1, 1.0f, false};
ValueSlideAnim g_animSetTime = {"", "", 1, 1.0f, false};
ValueSlideAnim g_animIdleTemp = {"", "", 1, 1.0f, false};
ValueSlideAnim g_animIdleFan = {"", "", 1, 1.0f, false};
NumberRailAnim g_animSetTempRail = {50.0f, 50.0f, 1.0f, false};
NumberRailAnim g_animIdleTempRail = {35.0f, 35.0f, 1.0f, false};

// 主要功能：获取当前预设。
// 使用方法：读取当前材料的风扇策略参数。
static const MaterialPreset& activePreset() { return PRESETS[g_presetIndex]; }

// 主要功能：获取当前首页/状态显示使用的材料标签。
// 使用方法：当参数处于自定义模式时返回 user(自定义)，否则返回预设名。
static const char* currentProfileName() {
  if (g_activeCustomPresetName.length() > 0) return g_activeCustomPresetName.c_str();
  return g_userCustomMode ? "user" : activePreset().name;
}

static uint8_t currentFanBasePct() {
  return g_userCustomMode ? g_customFanBasePct : activePreset().fanBasePct;
}

static uint8_t currentFanMaxPct() {
  return g_userCustomMode ? g_customFanMaxPct : activePreset().fanMaxPct;
}

// 前置声明：用于分组辅助函数中调用。
static void applyPreset(size_t presetIndex);
static void startDrying();
static void stopDrying();
static void startPidAutoTune();
static void clearFaults();
static void saveState(bool force);
static const char* currentProfileName();
static String faultFlagsToText(uint32_t flags);
static const char* faultFlagsToShortText(uint32_t flags);
static void setFault(FaultFlags fault, const char* reason);
static uint8_t currentFanBasePct();
static uint8_t currentFanMaxPct();
static String mqttControlTopic();
static String mqttStatusTopic();
static String mqttControlPushTopic();
static String mqttStatusPushTopic();
static bool applyControlJson(JsonDocument& in, String& errMsg);
static void fillStatusJson(JsonDocument& doc);
static bool checkWebToken(bool allowAnonymous = false);
static void markStatusDirty();
static void clearActiveCustomPreset();
static void loadUserPresets();
static bool saveUserPresets();
static int findUserPresetById(const String& id);
static bool applyUserPresetById(const String& id);
static bool upsertUserPresetFromJson(JsonDocument& in, String& errMsg);
static bool deleteUserPresetById(const String& id);
static bool buildHistorySummary(HistorySummary& summary);
static bool sendBemfaNotification(const char* kind, const String& message);
static void mqttCallback(char* topic, uint8_t* payload, unsigned int length);
static void serviceMqtt();
static void publishStatusMqtt(bool force);
static bool checkBemfaOta(bool applyUpdate, String& resultMsg);
static String maskSecret(const String& value);
static void printBemfaMqttConfig(const char* reason);
static void printBemfaMqttDisabledReason();
static void trackHumidityHistory(uint32_t nowMs);
static void updateDryingCompletionByHumidity();
static void updateAdvancedFaults(uint32_t nowMs);
static String urlEncode(const String& value);

// 前置声明：网络与 Web 控制相关函数。
static String formatDurationForWeb(uint32_t sec);
static void drawWifiStatusScreen(const char* title, const char* detail);
static void loadWifiConfig();
static void saveWifiConfig(const String& ssid, const String& pass);
static void clearWifiConfig();
static bool connectWifiSta();
static void startConfigApPortal();
static void startWebServer();
static void handleWebRoot();
static void handleApiStatus();
static void handleApiPresets();
static void handleApiControl();
static void handleApiWifi();
static void handleApiDevcmd();
static void handleApiSettings();
static void handleApiCustomPresets();
static void handleApiCustomPresetsDelete();
static void handleApiHistory();
static void handleApiLogDownload();
static void handleApiOta();
static void handleNotFound();
static void serviceNetwork();
static void setFanPct(uint8_t pct);
static String executeConsoleCommand(const String& cmdLine);
static void handleButtonEvents(bool up, bool down, bool left, bool right, bool ok);
static const char* sensorTypeToString(SensorType type);
static bool initTempHumiditySensor();
static bool readTempHumidity(float& outTempC, float& outHumiPct);
static bool idleHeatEnabled();
static void drawHomeLayer(int16_t yOffset);
static void updateSettingAnimations(uint32_t nowMs);
static void startValueSlide(ValueSlideAnim& anim, const char* oldText, const char* newText, int8_t dir);
static void drawFocusCard(int16_t baselineY, int16_t width, int16_t height, int16_t radius);
static void drawCenteredText(const char* text, int16_t baselineY);
static void drawValueWithSlide(const ValueSlideAnim& anim, const char* currentText, int16_t x, int16_t y,
                               int16_t deltaY);
static float easeOutCubic(float t);
static void startNumberRailAnim(NumberRailAnim& anim, float oldValue, float newValue);
static float sampleNumberRail(const NumberRailAnim& anim, float fallbackValue);
static void drawTemperatureRail(const NumberRailAnim& anim, float currentValue, float minValue, float maxValue);
static void formatSetTempText(float value, char* out, size_t outSize);
static void formatSetTimeText(uint32_t sec, char* out, size_t outSize);
static void formatIdleTempText(float value, char* out, size_t outSize);
static void formatIdleFanText(uint8_t pct, char* out, size_t outSize);

// 主要功能：根据预设索引查询其所属材料组。
// 使用方法：用于预设菜单分组筛选与显示。
static uint8_t presetGroupOf(size_t presetIndex) {
  if (presetIndex >= PRESET_COUNT) return GROUP_BASIC;
  return PRESET_GROUPS[presetIndex];
}

// 主要功能：在指定分组内查找下一个预设索引。
// 使用方法：delta=+1 向后切换，delta=-1 向前切换；若组内无项则返回当前值。
static size_t nextPresetInGroup(size_t current, int8_t delta, uint8_t group) {
  if (PRESET_COUNT == 0) return 0;
  for (size_t step = 0; step < PRESET_COUNT; ++step) {
    if (delta > 0) {
      current = (current + 1) % PRESET_COUNT;
    } else {
      current = (current == 0) ? (PRESET_COUNT - 1) : (current - 1);
    }
    if (presetGroupOf(current) == group) return current;
  }
  return current;
}

// 主要功能：判断当前是否开启闲时保温功能。
// 使用方法：仅在非烘干状态且闲时温度>=最低可控温度时返回 true。
static bool idleHeatEnabled() { return (!g_dryingActive && g_idleTempC >= TARGET_TEMP_MIN_C); }

// 主要功能：切换到某个分组并定位到该分组内最近可用预设。
// 使用方法：在预设菜单切换组时调用。
static void switchPresetGroup(uint8_t group, int8_t directionHint) {
  if (group >= GROUP_COUNT) return;
  g_presetGroupFilter = group;
  if (presetGroupOf(g_presetIndex) != g_presetGroupFilter) {
    g_presetIndex = nextPresetInGroup(g_presetIndex, directionHint, g_presetGroupFilter);
    applyPreset(g_presetIndex);
  }
}

// 主要功能：按名称查找预设索引。
// 使用方法：串口命令或菜单中按名称匹配预设。
static int findPresetByName(const String& name) {
  String upper = name;
  upper.toUpperCase();
  for (size_t i = 0; i < PRESET_COUNT; ++i) {
    String n = PRESETS[i].name;
    n.toUpperCase();
    if (n == upper) return static_cast<int>(i);
  }
  return -1;
}

// 主要功能：把秒数格式化为 HH:MM:SS 文本（用于网页显示）。
// 使用方法：传入剩余秒数，返回标准时分秒字符串。
static String formatDurationForWeb(uint32_t sec) {
  char buf[16];
  FormatDuration(sec, buf, sizeof(buf));
  return String(buf);
}

static void formatSetTempText(float value, char* out, size_t outSize) {
  snprintf(out, outSize, "T=%.1fC", value);
}

static void formatSetTimeText(uint32_t sec, char* out, size_t outSize) {
  FormatDuration(sec, out, outSize);
}

static void formatIdleTempText(float value, char* out, size_t outSize) {
  if (value < TARGET_TEMP_MIN_C) {
    snprintf(out, outSize, "OFF");
  } else {
    snprintf(out, outSize, "T=%.1fC", value);
  }
}

static void formatIdleFanText(uint8_t pct, char* out, size_t outSize) {
  snprintf(out, outSize, "FAN=%u%%", pct);
}

static float easeOutCubic(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  float oneMinus = 1.0f - t;
  return 1.0f - oneMinus * oneMinus * oneMinus;
}

static void startNumberRailAnim(NumberRailAnim& anim, float oldValue, float newValue) {
  if (fabsf(oldValue - newValue) < 0.001f) return;
  anim.oldValue = oldValue;
  anim.newValue = newValue;
  anim.progress = 0.0f;
  anim.active = true;
}

static float sampleNumberRail(const NumberRailAnim& anim, float fallbackValue) {
  if (!anim.active) return fallbackValue;
  float eased = easeOutCubic(anim.progress);
  return anim.oldValue + (anim.newValue - anim.oldValue) * eased;
}

static void startValueSlide(ValueSlideAnim& anim, const char* oldText, const char* newText, int8_t dir) {
  if (strcmp(oldText, newText) == 0) return;
  snprintf(anim.oldText, sizeof(anim.oldText), "%s", oldText);
  snprintf(anim.newText, sizeof(anim.newText), "%s", newText);
  anim.dir = (dir >= 0) ? 1 : -1;
  anim.progress = 0.0f;
  anim.active = true;
}

static void updateSettingAnimations(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  if (lastMs == 0) {
    lastMs = nowMs;
    return;
  }
  float dtSec = static_cast<float>(nowMs - lastMs) / 1000.0f;
  lastMs = nowMs;
  if (dtSec < 0.001f) dtSec = 0.001f;
  if (dtSec > 0.05f) dtSec = 0.05f;

  ValueSlideAnim* anims[] = {&g_animSetTemp, &g_animSetTime, &g_animIdleTemp, &g_animIdleFan};
  for (ValueSlideAnim* anim : anims) {
    if (!anim->active) continue;
    anim->progress += dtSec * 6.5f;
    if (anim->progress >= 1.0f) {
      anim->progress = 1.0f;
      anim->active = false;
    }
  }

  if (g_animSetTempRail.active) {
    g_animSetTempRail.progress += dtSec * 4.8f;
    if (g_animSetTempRail.progress >= 1.0f) {
      g_animSetTempRail.progress = 1.0f;
      g_animSetTempRail.active = false;
    }
  }

  if (g_animIdleTempRail.active) {
    g_animIdleTempRail.progress += dtSec * 4.8f;
    if (g_animIdleTempRail.progress >= 1.0f) {
      g_animIdleTempRail.progress = 1.0f;
      g_animIdleTempRail.active = false;
    }
  }
}

static void drawFocusCard(int16_t baselineY, int16_t width, int16_t height, int16_t radius) {
  const int16_t centerX = 64;
  int16_t x = centerX - width / 2;
  int16_t y = baselineY - height + 2;
  oled.drawRBox(x, y, width, height, radius);
}

static void drawCenteredText(const char* text, int16_t baselineY) {
  const int16_t centerX = 64;
  int16_t width = oled.getStrWidth(text);
  oled.drawStr(centerX - width / 2, baselineY, text);
}

static void drawValueWithSlide(const ValueSlideAnim& anim, const char* currentText, int16_t x, int16_t y,
                               int16_t deltaY) {
  (void)x;
  const int16_t centerX = 64;
  const int16_t focusW = 98;
  const int16_t focusH = 32;
  const int16_t focusX = centerX - focusW / 2;
  const int16_t focusY = y - focusH + 2;
  drawFocusCard(y, focusW, focusH, 6);

  oled.setDrawColor(0);
  oled.setClipWindow(focusX + 3, focusY + 2, focusX + focusW - 3, focusY + focusH - 2);
  if (!anim.active) {
    drawCenteredText(currentText, y);
    oled.setMaxClipWindow();
    oled.setDrawColor(1);
    return;
  }

  int16_t oldY = y + ((anim.dir > 0) ? static_cast<int16_t>(-anim.progress * deltaY)
                                      : static_cast<int16_t>(anim.progress * deltaY));
  int16_t newY =
      y + ((anim.dir > 0) ? static_cast<int16_t>((1.0f - anim.progress) * deltaY)
                          : static_cast<int16_t>(-(1.0f - anim.progress) * deltaY));
  int16_t top = y - deltaY - 2;
  int16_t bottom = y + deltaY + 2;
  if (top < 0) top = 0;
  if (bottom > 63) bottom = 63;

  int16_t clipTop = (focusY + 2 > top) ? (focusY + 2) : top;
  int16_t clipBottom = (focusY + focusH - 2 < bottom + 1) ? (focusY + focusH - 2) : (bottom + 1);
  oled.setClipWindow(focusX + 3, clipTop, focusX + focusW - 3, clipBottom);
  drawCenteredText(anim.oldText, oldY);
  drawCenteredText(anim.newText, newY);
  oled.setMaxClipWindow();
  oled.setDrawColor(1);
}

static void drawTemperatureRail(const NumberRailAnim& anim, float currentValue, float minValue, float maxValue) {
  const int16_t centerX = 64;
  const int16_t baselineY = 52;
  float displayValue = ClampF(sampleNumberRail(anim, currentValue), minValue, maxValue);
  const int16_t focusW = 82;
  const int16_t focusH = 32;
  drawFocusCard(baselineY, focusW, focusH, 6);

  oled.setDrawColor(0);
  oled.setFont(u8g2_font_logisoso24_tf);
  char centerText[12];
  snprintf(centerText, sizeof(centerText), "%.1f", displayValue);
  int16_t centerWidth = oled.getStrWidth(centerText);
  int16_t centerTextX = centerX - centerWidth / 2 - 3;
  oled.drawStr(centerTextX, baselineY, centerText);

  oled.setFont(u8g2_font_6x12_tf);
  oled.drawStr(centerTextX + centerWidth + 4, baselineY - 1, "C");
  oled.setDrawColor(1);
  oled.setMaxClipWindow();
}

// 主要功能：在 OLED 上显示当前 Wi-Fi 阶段状态，避免联网阶段黑屏。
// 使用方法：连接路由器/启动 AP 配网时调用。
static String mqttControlTopic() {
  if (g_bemfaControlTopic.length() == 0) return "";
  return g_bemfaControlTopic;
}

static String mqttStatusTopic() {
  if (g_bemfaStatusTopic.length() == 0) return "";
  return g_bemfaStatusTopic;
}

static String mqttControlPushTopic() {
  if (g_bemfaControlTopic.length() == 0) return "";
  return g_bemfaControlTopic + "/set";
}

static String mqttStatusPushTopic() {
  if (g_bemfaStatusTopic.length() == 0) return "";
  return g_bemfaStatusTopic + "/set";
}

static String faultFlagsToText(uint32_t flags) {
  if (flags == FAULT_NONE) return "无故障";

  String text;
  auto appendFault = [&](const char* label) {
    if (text.length() > 0) text += " / ";
    text += label;
  };

  if (flags & FAULT_SENSOR) appendFault("传感器故障");
  if (flags & FAULT_OVER_TEMP) appendFault("温度过高");
  if (flags & FAULT_HEAT_TIMEOUT) appendFault("升温超时");
  if (flags & FAULT_HEATER_STUCK) appendFault("疑似加热器粘连");

  if (text.length() == 0) {
    text = String("未知故障(") + static_cast<unsigned long>(flags) + ")";
  }
  return text;
}

static const char* faultFlagsToShortText(uint32_t flags) {
  if (flags == FAULT_NONE) return "NONE";
  if ((flags & (flags - 1)) != 0) return "MULTI";
  if (flags & FAULT_SENSOR) return "SENSOR";
  if (flags & FAULT_OVER_TEMP) return "OVER TEMP";
  if (flags & FAULT_HEAT_TIMEOUT) return "HEAT TIMEOUT";
  if (flags & FAULT_HEATER_STUCK) return "HEATER STUCK";
  return "UNKNOWN";
}

static bool isBemfaTopicBaseValid(const String& topic) {
  if (topic.length() == 0) return false;
  for (size_t i = 0; i < topic.length(); ++i) {
    char c = topic[i];
    if (!isalnum(static_cast<unsigned char>(c))) return false;
  }
  return true;
}

static String maskSecret(const String& value) {
  if (value.length() == 0) return "(empty)";
  if (value.length() <= 8) return String("***len=") + value.length();
  return value.substring(0, 4) + "..." + value.substring(value.length() - 4);
}

static void printBemfaMqttConfig(const char* reason) {
  Serial.printf("[MQTT] %s | host=%s:%u uid=%s key=%s(len=%u) ctl=%s sta=%s ota=%s\n", reason, BEMFA_HOST,
                static_cast<unsigned>(BEMFA_MQTT_PORT), g_bemfaUid.c_str(), maskSecret(g_bemfaMqttKey).c_str(),
                static_cast<unsigned>(g_bemfaMqttKey.length()), g_bemfaControlTopic.c_str(), g_bemfaStatusTopic.c_str(),
                g_bemfaOtaTopic.length() > 0 ? g_bemfaOtaTopic.c_str() : "(empty)");
}

static void printBemfaMqttDisabledReason() {
  Serial.printf("[MQTT] disabled | wifi=%s key=%s ctl=%s sta=%s\n", g_wifiConnected ? "ok" : "no",
                g_bemfaMqttKey.length() > 0 ? "ok" : "empty",
                isBemfaTopicBaseValid(g_bemfaControlTopic) ? g_bemfaControlTopic.c_str() : "invalid",
                isBemfaTopicBaseValid(g_bemfaStatusTopic) ? g_bemfaStatusTopic.c_str() : "invalid");
}

static void markStatusDirty() { g_statusPublishDirty = true; }

static void clearActiveCustomPreset() {
  g_activeCustomPresetId = "";
  g_activeCustomPresetName = "";
}

static String urlEncode(const String& value) {
  static const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(value.length() * 3);
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

static bool checkWebToken(bool allowAnonymous) {
  if (g_webToken.length() == 0) return true;
  String token = g_webServer.hasArg("token") ? g_webServer.arg("token") : g_webServer.header("X-Dryer-Token");
  token.trim();
  if (token == g_webToken) return true;
  if (allowAnonymous) return false;
  g_webServer.send(401, "application/json", "{\"ok\":false,\"msg\":\"token required\"}");
  return false;
}

static void loadUserPresets() {
  g_userPresetCount = 0;
  for (uint8_t i = 0; i < MAX_USER_PRESETS; ++i) g_userPresets[i] = UserPreset();
  if (!SPIFFS.exists("/user_presets.json")) return;

  File f = SPIFFS.open("/user_presets.json", FILE_READ);
  if (!f) return;
  JsonDocument doc;
  if (deserializeJson(doc, f) != DeserializationError::Ok) {
    f.close();
    return;
  }
  f.close();

  JsonArray arr = doc["presets"].as<JsonArray>();
  if (arr.isNull()) return;

  for (JsonObject obj : arr) {
    if (g_userPresetCount >= MAX_USER_PRESETS) break;
    UserPreset& p = g_userPresets[g_userPresetCount++];
    p.used = true;
    snprintf(p.id, sizeof(p.id), "%s", obj["id"] | "");
    snprintf(p.name, sizeof(p.name), "%s", obj["name"] | "custom");
    p.targetTempC = ClampF(obj["target_c"] | 50.0f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
    p.durationSec = constrain(static_cast<uint32_t>((obj["duration_min"] | 240) * 60UL), DURATION_MIN_SEC, DURATION_MAX_SEC);
    p.fanBasePct = constrain(obj["fan_base_pct"] | 30, 0, 100);
    p.fanMaxPct = constrain(obj["fan_max_pct"] | 80, 0, 100);
    if (p.fanMaxPct < p.fanBasePct) p.fanMaxPct = p.fanBasePct;
    float idleTemp = obj["idle_temp_c"] | 0.0f;
    p.idleTempC = (idleTemp < TARGET_TEMP_MIN_C) ? IDLE_TEMP_DISABLED_C : ClampF(idleTemp, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
    p.idleFanPct = constrain(obj["idle_fan_pct"] | 0, 0, 100);
    p.humidityStopPct = ClampF(obj["humidity_stop_pct"] | 0.0f, 0.0f, 100.0f);
  }
}

static bool saveUserPresets() {
  JsonDocument doc;
  JsonArray arr = doc["presets"].to<JsonArray>();
  for (uint8_t i = 0; i < g_userPresetCount; ++i) {
    const UserPreset& p = g_userPresets[i];
    if (!p.used) continue;
    JsonObject obj = arr.add<JsonObject>();
    obj["id"] = p.id;
    obj["name"] = p.name;
    obj["target_c"] = p.targetTempC;
    obj["duration_min"] = p.durationSec / 60UL;
    obj["fan_base_pct"] = p.fanBasePct;
    obj["fan_max_pct"] = p.fanMaxPct;
    obj["idle_temp_c"] = p.idleTempC;
    obj["idle_fan_pct"] = p.idleFanPct;
    obj["humidity_stop_pct"] = p.humidityStopPct;
  }
  File f = SPIFFS.open("/user_presets.json", FILE_WRITE);
  if (!f) return false;
  serializeJson(doc, f);
  f.close();
  return true;
}

static int findUserPresetById(const String& id) {
  if (id.length() == 0) return -1;
  for (uint8_t i = 0; i < g_userPresetCount; ++i) {
    if (g_userPresets[i].used && id == g_userPresets[i].id) return static_cast<int>(i);
  }
  return -1;
}

static bool applyUserPresetById(const String& id) {
  int idx = findUserPresetById(id);
  if (idx < 0) return false;
  const UserPreset& p = g_userPresets[idx];
  g_userCustomMode = true;
  g_targetTempC = p.targetTempC;
  g_configDurationSec = p.durationSec;
  g_idleTempC = p.idleTempC;
  g_idleFanPct = p.idleFanPct;
  g_customFanBasePct = p.fanBasePct;
  g_customFanMaxPct = p.fanMaxPct;
  g_humidityStopPct = p.humidityStopPct;
  g_activeCustomPresetId = p.id;
  g_activeCustomPresetName = p.name;
  if (g_dryingActive) g_remainingSec = g_configDurationSec;
  markStatusDirty();
  return true;
}

static bool upsertUserPresetFromJson(JsonDocument& in, String& errMsg) {
  if (!in["name"].is<const char*>()) {
    errMsg = "name required";
    return false;
  }
  String name = in["name"].as<String>();
  name.trim();
  if (name.length() == 0) {
    errMsg = "name required";
    return false;
  }

  String id = in["id"].is<const char*>() ? in["id"].as<String>() : "";
  int idx = findUserPresetById(id);
  if (idx < 0 && g_userPresetCount >= MAX_USER_PRESETS) {
    errMsg = "preset full";
    return false;
  }
  if (idx < 0) idx = g_userPresetCount++;

  UserPreset& p = g_userPresets[idx];
  if (!p.used) {
    p = UserPreset();
    p.used = true;
    String newId = (id.length() > 0) ? id : String(millis(), HEX);
    snprintf(p.id, sizeof(p.id), "%s", newId.c_str());
  }

  snprintf(p.name, sizeof(p.name), "%s", name.c_str());
  p.targetTempC = ClampF(in["target_c"] | g_targetTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  p.durationSec = constrain(static_cast<uint32_t>((in["duration_min"] | (g_configDurationSec / 60UL)) * 60UL), DURATION_MIN_SEC, DURATION_MAX_SEC);
  p.fanBasePct = constrain(in["fan_base_pct"] | g_customFanBasePct, 0, 100);
  p.fanMaxPct = constrain(in["fan_max_pct"] | g_customFanMaxPct, 0, 100);
  if (p.fanMaxPct < p.fanBasePct) p.fanMaxPct = p.fanBasePct;
  float idleTemp = in["idle_temp_c"] | g_idleTempC;
  p.idleTempC = (idleTemp < TARGET_TEMP_MIN_C) ? IDLE_TEMP_DISABLED_C : ClampF(idleTemp, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  p.idleFanPct = constrain(in["idle_fan_pct"] | g_idleFanPct, 0, 100);
  p.humidityStopPct = ClampF(in["humidity_stop_pct"] | g_humidityStopPct, 0.0f, 100.0f);
  return saveUserPresets();
}

static bool deleteUserPresetById(const String& id) {
  int idx = findUserPresetById(id);
  if (idx < 0) return false;
  for (int i = idx; i < g_userPresetCount - 1; ++i) g_userPresets[i] = g_userPresets[i + 1];
  if (g_userPresetCount > 0) g_userPresetCount--;
  if (g_activeCustomPresetId == id) clearActiveCustomPreset();
  if (g_userPresetCount < MAX_USER_PRESETS) g_userPresets[g_userPresetCount] = UserPreset();
  return saveUserPresets();
}

static bool buildHistorySummary(HistorySummary& summary) {
  if (!SPIFFS.exists("/dryer_log.csv")) return false;
  File f = SPIFFS.open("/dryer_log.csv", FILE_READ);
  if (!f) return false;

  bool first = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("ms,")) continue;
    unsigned long ms = 0, fault = 0, remain = 0;
    float temp = 0.0f, humi = 0.0f, target = 0.0f;
    unsigned heater = 0, fan = 0, active = 0;
    char preset[32] = {0};
    int n = sscanf(line.c_str(), "%lu,%f,%f,%f,%u,%u,%u,%lu,%31[^,],%lu", &ms, &temp, &humi, &target,
                   &heater, &fan, &active, &fault, preset, &remain);
    if (n < 10) continue;

    if (first) {
      summary.minTempC = summary.maxTempC = temp;
      summary.minHumiPct = summary.maxHumiPct = humi;
      first = false;
    } else {
      if (temp < summary.minTempC) summary.minTempC = temp;
      if (temp > summary.maxTempC) summary.maxTempC = temp;
      if (humi < summary.minHumiPct) summary.minHumiPct = humi;
      if (humi > summary.maxHumiPct) summary.maxHumiPct = humi;
    }
    summary.rows++;
    summary.avgTempC += temp;
    summary.avgHumiPct += humi;
    summary.avgHeaterPct += heater;
    summary.avgFanPct += fan;
    if (active > 0) summary.activeRows++;
  }
  f.close();
  if (summary.rows == 0) return false;
  summary.avgTempC /= summary.rows;
  summary.avgHumiPct /= summary.rows;
  summary.avgHeaterPct /= summary.rows;
  summary.avgFanPct /= summary.rows;
  return true;
}

static void drawWifiStatusScreen(const char* title, const char* detail) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 14, "网络状态");
  oled.drawHLine(0, 16, 128);
  oled.setFont(u8g2_font_6x12_tf);
  oled.drawStr(0, 34, title);
  oled.drawStr(0, 50, detail);
  oled.sendBuffer();
}

// 主要功能：从 NVS 读取 Wi-Fi 凭据。
// 使用方法：setup 阶段调用一次，后续由 connectWifiSta() 使用。
static void loadWifiConfig() {
  g_wifiSsid = prefs.getString("wifi_ssid", "");
  g_wifiPass = prefs.getString("wifi_pass", "");
}

// 主要功能：将 Wi-Fi 凭据写入 NVS。
// 使用方法：网页提交新的 SSID/密码后调用并持久化。
static void saveWifiConfig(const String& ssid, const String& pass) {
  g_wifiSsid = ssid;
  g_wifiPass = pass;
  prefs.putString("wifi_ssid", g_wifiSsid);
  prefs.putString("wifi_pass", g_wifiPass);
}

// 主要功能：清空已保存的 Wi-Fi 凭据。
// 使用方法：串口调试时调用，清空后设备会自动进入 AP 配网模式。
static void clearWifiConfig() {
  g_wifiSsid = "";
  g_wifiPass = "";
  prefs.remove("wifi_ssid");
  prefs.remove("wifi_pass");
}

// 主要功能：使用已保存凭据连接路由器（STA）。
// 使用方法：返回 true 表示联网成功，false 表示失败并保持离线。
static bool connectWifiSta() {
  if (g_wifiSsid.length() == 0) return false;

  drawWifiStatusScreen("WiFi connecting...", g_wifiSsid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_wifiSsid.c_str(), g_wifiPass.c_str());
  uint32_t startMs = millis();
  uint32_t lastDrawMs = 0;
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
    if (millis() - lastDrawMs >= 500) {
      lastDrawMs = millis();
      char buf[24];
      uint32_t elapsed = static_cast<uint32_t>(millis() - startMs);
      if (elapsed > WIFI_CONNECT_TIMEOUT_MS) elapsed = WIFI_CONNECT_TIMEOUT_MS;
      uint32_t left = (WIFI_CONNECT_TIMEOUT_MS - elapsed) / 1000;
      snprintf(buf, sizeof(buf), "timeout in %lus", static_cast<unsigned long>(left));
      drawWifiStatusScreen("WiFi connecting...", buf);
    }
    delay(200);
  }

  g_wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (g_wifiConnected) {
    g_apConfigMode = false;
    Serial.print("WiFi 已连接，IP: ");
    Serial.println(WiFi.localIP());
    drawWifiStatusScreen("WiFi connected", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi 连接失败，进入配网模式。");
    drawWifiStatusScreen("WiFi failed", "switching to AP...");
  }
  return g_wifiConnected;
}

// 主要功能：开启 AP 配网页（含 DNS 劫持）。
// 使用方法：无凭据或 STA 失败时调用，手机连接热点后访问任意网页即可进入配置页。
static void startConfigApPortal() {
  g_dnsServer.stop();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  g_dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  g_apConfigMode = true;
  g_wifiConnected = (WiFi.status() == WL_CONNECTED);
  Serial.print("配网热点已开启，SSID: ");
  Serial.print(WIFI_AP_SSID);
  Serial.print(" IP: ");
  Serial.println(WiFi.softAPIP());
  drawWifiStatusScreen("AP config mode", WiFi.softAPIP().toString().c_str());
}

static const char WEB_ROOT_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>3D耗材烘干箱</title>
<style>
:root{--bg:#edf2f7;--card:#fff;--line:#d7dee7;--txt:#0f172a;--muted:#64748b;--ok:#166534;--bad:#b91c1c;--brand:#0f766e}*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#f8fafc,#edf2f7);color:var(--txt);font-family:"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif}.wrap{max-width:1500px;margin:0 auto;padding:16px}.head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:14px}.head h1{margin:0;font-size:24px}.muted{color:var(--muted)}.ok{color:var(--ok)}.bad{color:var(--bad)}.grid{display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:12px}.s12{grid-column:span 12}.s6{grid-column:span 6}.s4{grid-column:span 4}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px;box-shadow:0 10px 30px rgba(15,23,42,.06)}.kpis{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}.kpi{padding:12px;border:1px solid var(--line);border-radius:12px;background:#f8fafc}.kpi .k{font-size:12px;color:var(--muted)}.kpi .v{font-size:22px;font-weight:700;margin-top:4px}h2{margin:0 0 10px;font-size:17px}.row{display:flex;gap:8px;flex-wrap:wrap}.two,.three{display:grid;gap:10px}.two{grid-template-columns:repeat(2,minmax(0,1fr))}.three{grid-template-columns:repeat(3,minmax(0,1fr))}label{display:block;font-size:13px;color:var(--muted);margin:8px 0 4px}input,select,button,textarea{width:100%;padding:9px 10px;border-radius:10px;border:1px solid #cbd5e1;background:#fff}button{cursor:pointer;background:#f8fafc}.primary{background:var(--brand);color:#fff;border-color:var(--brand)}.warnBtn{background:#fff7ed;border-color:#fdba74}.badBtn{background:#fef2f2;border-color:#fca5a5}.msg{margin:0 0 12px;padding:10px 12px;border-radius:12px;background:#e2e8f0;min-height:40px}textarea{font-family:Consolas,"Courier New",monospace;resize:vertical}.pre{white-space:pre-wrap;word-break:break-word;background:#0f172a;color:#e2e8f0;padding:10px;border-radius:12px;min-height:120px}.pill{display:inline-block;padding:3px 8px;border-radius:999px;background:#e2e8f0;font-size:12px;margin-left:6px}.mono{font-family:Consolas,"Courier New",monospace}canvas{width:100%;height:170px;border:1px solid var(--line);border-radius:12px;background:#fff}@media (max-width:1120px){.s6,.s4{grid-column:span 12}.kpis,.two,.three{grid-template-columns:repeat(1,minmax(0,1fr))}}
</style></head><body><div class="wrap">
<div class="head"><div><h1>3D打印耗材烘干箱</h1><div id="net" class="muted">网络状态加载中...</div></div><div class="muted">Web 控制台<span id="authBadge" class="pill">鉴权检测中</span></div></div>
<div id="msg" class="msg">页面初始化中...</div>
<div class="grid">
<div class="card s12"><div class="kpis">
<div class="kpi"><div class="k">温度</div><div class="v"><span id="temp">--</span> C</div></div>
<div class="kpi"><div class="k">湿度</div><div class="v"><span id="humi">--</span> %</div></div>
<div class="kpi"><div class="k">目标 / 剩余</div><div class="v"><span id="target">--</span> C / <span id="remain">--</span></div></div>
<div class="kpi"><div class="k">状态 / 配方</div><div class="v"><span id="active">--</span> / <span id="preset">--</span></div></div>
</div><div class="muted" style="margin-top:10px">加热 <span id="heater">--</span>% | 风扇 <span id="fanPct">--</span>% | 估算转速 <span id="fan">--</span> RPM | PWM 平均电压 <span id="fanSigV">--</span> V | 故障 <span id="fault" class="bad">无</span> | MQTT <span id="mqttState">--</span> | 通知 <span id="notifyState">--</span> | OTA <span id="otaState">--</span></div></div>
<div class="card s6"><h2>温度曲线</h2><canvas id="cTemp" width="640" height="170"></canvas></div>
<div class="card s6"><h2>湿度曲线</h2><canvas id="cHumi" width="640" height="170"></canvas></div>
<div class="card s6"><h2>风扇估算转速</h2><canvas id="cFan" width="640" height="170"></canvas></div>
<div class="card s6"><h2>加热输出</h2><canvas id="cHeat" width="640" height="170"></canvas></div>
<div class="card s4"><h2>控制</h2>
<label>模式</label><select id="modeSel" onchange="updateModeUi()"><option value="preset">内置预设</option><option value="custom">自定义参数</option><option value="custom_preset">用户预设</option></select>
<div id="builtinWrap"><label>内置预设</label><select id="presetSel"></select></div>
<div id="customPresetWrap"><label>用户预设</label><select id="customPresetSel"></select><div class="row" style="margin-top:8px"><button onclick="applySelectedCustomPreset()">载入用户预设</button><button class="badBtn" onclick="deleteCustomPreset()">删除</button></div></div>
<label>目标温度 (35~120 C)</label><input id="setTemp" type="number" step="0.5" min="35" max="120">
<label>烘干时长 (分钟)</label><input id="setDur" type="number" min="30" max="1440">
<div class="two"><div><label>闲时温度 (0=关闭)</label><input id="setIdleTemp" type="number" step="0.5" min="0" max="120"></div><div><label>空闲风扇 (%)</label><input id="setIdleFan" type="number" min="0" max="100"></div></div>
<div class="two"><div><label>自定义风扇基础 (%)</label><input id="fanBase" type="number" min="0" max="100"></div><div><label>自定义风扇上限 (%)</label><input id="fanMax" type="number" min="0" max="100"></div></div>
<div class="two"><div><label>湿度阈值停机 (%)</label><input id="humiStop" type="number" step="0.1" min="0" max="100"></div><div><label>湿度趋稳阈值 Δ(%)</label><input id="humiStable" type="number" step="0.1" min="0" max="10"></div></div>
<div class="row" style="margin-top:10px"><button class="primary" onclick="sendCtrl('start')">启动</button><button onclick="sendCtrl('stop')">停止</button><button class="warnBtn" onclick="applyCfg()">应用参数</button><button class="badBtn" onclick="sendCtrl('faultreset')">清故障</button></div></div>
<div class="card s4"><h2>PID 与保护</h2>
<div class="three"><div><label>Kp</label><input id="setKp" type="number" step="0.001" min="0" max="10"></div><div><label>Ki</label><input id="setKi" type="number" step="0.0001" min="0" max="1"></div><div><label>Kd</label><input id="setKd" type="number" step="0.001" min="0" max="20"></div></div>
<div class="row" style="margin-top:10px"><button class="primary" onclick="applyPid()">保存 PID</button><button onclick="sendCtrl('autotune')">自动整定</button><button onclick="sendCtrl('pidreset')">恢复默认</button></div>
<div class="muted" style="margin-top:10px">状态 <span id="tuneState">IDLE</span> | 进度 <span id="tuneProg">0</span>%<br>阶段 <span id="tuneStage">待机</span> | 拟合 <span id="tuneFit">未开始</span><br>样本 A/Tu: <span id="tuneAmpN">0</span>/<span id="tuneTuN">0</span> | A=<span id="tuneAmp">--</span> C | Tu=<span id="tuneTu">--</span> s<br>预估 Kp/Ki/Kd: <span id="tuneEst">--</span><br>保护: 湿度阈值停机 + 湿度趋稳停机 + 升温超时 + 疑似加热器粘连</div>
<label style="margin-top:10px">当前通知结果</label><div id="notifyDetail" class="pre"></div></div>
<div class="card s4"><h2>云设置与鉴权</h2>
<label>当前访问 Token</label><div class="row"><input id="accessToken" type="password" placeholder="仅保存在浏览器本地" style="flex:1"><button onclick="saveAccessToken()" style="width:130px">保存 Token</button><button onclick="clearAccessToken()" style="width:110px">清空</button></div>
<div class="two"><div><label>Bemfa UID</label><input id="mqttUid" type="text"></div><div><label>Bemfa 私钥</label><input id="mqttKey" type="text" placeholder="MQTT ClientId"></div></div>
<div class="two"><div><label>通知分组</label><input id="notifyGroup" type="text" placeholder="default"></div><div><label>MQTT 连接规则</label><input value="私钥作 ClientId，用户名密码留空" readonly></div></div>
<label>控制主题基名</label><input id="mqttCtlBase" type="text" placeholder="dryer010">
<label>状态主题基名</label><input id="mqttStaBase" type="text" placeholder="dryer004">
<label>OTA 主题基名</label><input id="otaBase" type="text" placeholder="dryerota01">
<div class="row" style="margin-top:8px"><label style="margin:0"><input id="notifyEnabled" type="checkbox" style="width:auto;margin-right:6px">启用微信通知</label><label style="margin:0"><input id="otaAuto" type="checkbox" style="width:auto;margin-right:6px">启用自动 OTA 检查</label></div>
<label>设置 / 更新设备 Web Token</label><input id="newToken" type="text" placeholder="留空则不修改">
<div class="row" style="margin-top:10px"><button class="primary" onclick="saveSettings()">保存云设置</button><button onclick="loadSettings()">重新读取</button></div>
<div class="muted" style="margin-top:10px">Bemfa MQTT 主题: 控制 <span id="mqttCtlTopic" class="mono">--</span>，状态 <span id="mqttStaTopic" class="mono">--</span></div></div>
<div class="card s4"><h2>用户预设</h2><label>名称</label><input id="customName" type="text" placeholder="例如 PLA-低湿度"><div class="row" style="margin-top:10px"><button class="primary" onclick="saveCustomPreset()">保存当前参数为预设</button><button onclick="loadCustomPresets()">刷新列表</button></div><div class="muted" style="margin-top:10px">保存内容包括温度、时长、闲时参数、自定义风扇和湿度阈值。</div><label style="margin-top:10px">预设列表</label><div id="customPresetList" class="pre"></div></div>
<div class="card s4"><h2>历史与 OTA</h2><div id="historyBox" class="pre"></div><div class="row" style="margin-top:10px"><button onclick="loadHistory()">刷新统计</button><button onclick="downloadLog()">下载 CSV 日志</button></div><div class="row" style="margin-top:10px"><button onclick="otaCheck()">检查 OTA</button><button class="warnBtn" onclick="otaApply()">执行 OTA</button></div><label style="margin-top:10px">OTA 信息</label><div id="otaBox" class="pre"></div></div>
<div class="card s4"><h2>Wi-Fi</h2><label>SSID</label><input id="ssid" type="text" placeholder="路由器名称"><label>密码</label><input id="pass" type="password" placeholder="路由器密码"><button class="primary" style="margin-top:10px" onclick="saveWifi()">保存并连接</button><div class="muted" style="margin-top:10px">AP 配网: Dryer-Config / 12345678 / 192.168.4.1</div><label style="margin-top:10px">原始状态 JSON</label><div id="rawJson" class="pre"></div></div>
<div class="card s12"><h2>开发接口</h2><div class="three"><div><label>温湿度传感器</label><select id="sensorType"><option value="aht10">AHT10</option><option value="sht3x">SHT3X</option></select></div><div><label>PWM 极性</label><select id="fanPwmPol"><option value="active_low">有效低电平</option><option value="active_high">有效高电平</option></select></div><div><label>虚拟按键</label><div class="row"><button onclick="sendDevKey('up')">上</button><button onclick="sendDevKey('down')">下</button><button onclick="sendDevKey('left')">左</button><button onclick="sendDevKey('right')">右</button><button onclick="sendDevKey('ok')">中</button></div></div></div><div class="row" style="margin-top:10px"><button onclick="applySensorType()">应用传感器</button><button onclick="applyFanPwmPol()">应用 PWM 极性</button><button onclick="sendCtrl('ota_check')">控制接口检查 OTA</button><button onclick="sendCtrl('ota_apply')">控制接口执行 OTA</button></div><label>命令输入</label><div class="row"><input id="devCmd" type="text" placeholder="help / status / start / stop / btn up" style="flex:1"><button onclick="sendDevCmd()" style="width:120px">发送</button></div><label>命令回显</label><textarea id="devLog" rows="8" readonly></textarea></div>
</div>
<script>
const HIS=180,hs={t:[],h:[],f:[],p:[]};let uiInited=false,lastPresetIndex=-1,lastCustomPresetId='',lastStatus=null;
function el(id){return document.getElementById(id)}function setText(id,v){el(id).textContent=v}function canFill(id){return document.activeElement!==el(id)}function token(){return localStorage.getItem('dryer_token')||''}
function isBemfaTopicBase(v){return /^[A-Za-z0-9]+$/.test(v)}
function authHeaders(extra){const h=Object.assign({},extra||{});const t=token();if(t)h['X-Dryer-Token']=t;return h}
async function api(url,opt){const cfg=Object.assign({headers:{}},opt||{});cfg.headers=authHeaders(cfg.headers);const r=await fetch(url,cfg);const text=await r.text();let data=null;try{data=text?JSON.parse(text):{}}catch{data={raw:text}}if(!r.ok)throw new Error((data&&data.msg)?data.msg:('HTTP '+r.status));return data}
async function jget(u){return await api(u)}async function jpost(u,d){return await api(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
function showMsg(msg,cls){const box=el('msg');box.textContent=msg;box.className='msg'+(cls?(' '+cls):'')}function fmt(v,d){return (typeof v==='number'&&isFinite(v))?v.toFixed(d):'--'}function push(arr,v){arr.push(v);if(arr.length>HIS)arr.shift()}
function mapTuneStage(s){const m={IDLE:'待机',HEATUP:'升温激励',SEARCH_AMP:'振幅采样',WAIT_PERIOD:'周期等待',COLLECT:'样本积累',FIT:'拟合计算',DONE:'完成',FAIL:'失败'};return m[s]||s||'--'}
function mapTuneFit(s){const m={N_A:'未开始',INSUFFICIENT:'样本不足',AMP_LOW:'振幅过小',PERIOD_SHORT:'周期偏短',PREVIEW:'可预估',READY:'可拟合',DONE:'拟合成功',FAIL:'拟合失败'};return m[s]||s||'--'}
function updateModeUi(){const mode=el('modeSel').value;el('builtinWrap').style.display=(mode==='preset')?'block':'none';el('customPresetWrap').style.display=(mode==='custom_preset')?'block':'none'}
function calcRange(data,opt){if(data.length===0)return {min:opt.defMin,max:opt.defMax};let mn=Math.min(...data),mx=Math.max(...data),span=Math.max(mx-mn,opt.minSpan),pad=span*opt.pad;let lo=mn-pad,hi=mx+pad;if(opt.hardMin!==null)lo=Math.max(opt.hardMin,lo);if(opt.hardMax!==null)hi=Math.min(opt.hardMax,hi);if((hi-lo)<opt.minSpan){const mid=(hi+lo)/2;lo=mid-opt.minSpan/2;hi=mid+opt.minSpan/2}return {min:lo,max:hi}}
function draw(canvasId,data,color,minV,maxV){const c=el(canvasId),ctx=c.getContext('2d'),w=c.width,h=c.height,p=18;ctx.clearRect(0,0,w,h);ctx.strokeStyle='#d7dee7';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(p,p);ctx.lineTo(p,h-p);ctx.lineTo(w-p,h-p);ctx.stroke();const mid=(minV+maxV)/2;ctx.fillStyle='#64748b';ctx.font='11px sans-serif';ctx.fillText(maxV.toFixed(1),2,p+2);ctx.fillText(mid.toFixed(1),2,h/2+3);ctx.fillText(minV.toFixed(1),2,h-p+3);if(data.length<2||Math.abs(maxV-minV)<0.0001)return;ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<data.length;i++){const x=p+(i*(w-2*p))/Math.max(1,HIS-1),y=h-p-((data[i]-minV)/(maxV-minV))*(h-2*p);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)}ctx.stroke()}
function renderCharts(){const rT=calcRange(hs.t,{hardMin:null,hardMax:null,defMin:20,defMax:60,minSpan:3,pad:0.2}),rH=calcRange(hs.h,{hardMin:0,hardMax:100,defMin:20,defMax:80,minSpan:8,pad:0.15}),rF=calcRange(hs.f,{hardMin:0,hardMax:null,defMin:0,defMax:3000,minSpan:300,pad:0.15}),rP=calcRange(hs.p,{hardMin:0,hardMax:100,defMin:0,defMax:100,minSpan:10,pad:0.1});draw('cTemp',hs.t,'#ef4444',rT.min,rT.max);draw('cHumi',hs.h,'#2563eb',rH.min,rH.max);draw('cFan',hs.f,'#16a34a',rF.min,rF.max);draw('cHeat',hs.p,'#f59e0b',rP.min,rP.max)}
function fillIfIdle(id,v){if(canFill(id)&&v!==undefined&&v!==null)el(id).value=v}
function saveAccessToken(){localStorage.setItem('dryer_token',el('accessToken').value.trim());showMsg('已保存本地访问 Token');loadSettings().then(refresh).catch(handleError)}
function clearAccessToken(){localStorage.removeItem('dryer_token');el('accessToken').value='';showMsg('已清空本地访问 Token','bad');loadSettings().then(refresh).catch(handleError)}
function handleError(err){showMsg(err.message||String(err),'bad')}
async function loadSettings(){const s=await jget('/api/settings');el('authBadge').textContent=s.auth_enabled?'已启用 Token':'未启用 Token';el('accessToken').value=token();if(s.mqtt_uid!==undefined){fillIfIdle('mqttUid',s.mqtt_uid||'');fillIfIdle('mqttKey',s.mqtt_key||'');fillIfIdle('mqttCtlBase',s.mqtt_control_topic_base||'');fillIfIdle('mqttStaBase',s.mqtt_status_topic_base||'');fillIfIdle('otaBase',s.ota_topic_base||'');fillIfIdle('notifyGroup',s.notify_group||'default');el('notifyEnabled').checked=!!s.notify_enabled;el('otaAuto').checked=!!s.ota_auto_check}}
async function saveSettings(){const ctl=el('mqttCtlBase').value.trim(),sta=el('mqttStaBase').value.trim(),ota=el('otaBase').value.trim();if(!isBemfaTopicBase(ctl)||!isBemfaTopicBase(sta)||(ota&&!isBemfaTopicBase(ota))){showMsg('Bemfa 主题基名只能包含字母和数字','bad');return}const p={mqtt_uid:el('mqttUid').value.trim(),mqtt_key:el('mqttKey').value.trim(),mqtt_control_topic_base:ctl,mqtt_status_topic_base:sta,ota_topic_base:ota,notify_enabled:el('notifyEnabled').checked,notify_group:el('notifyGroup').value.trim(),ota_auto_check:el('otaAuto').checked};const nt=el('newToken').value.trim();if(nt)p.token=nt;await jpost('/api/settings',p);if(nt){localStorage.setItem('dryer_token',nt);el('accessToken').value=nt;el('newToken').value=''}showMsg('云设置已保存');await loadSettings();await refresh()}
async function loadPresets(){const p=await jget('/api/presets'),a=el('presetSel'),b=el('customPresetSel');a.innerHTML='';(p.presets||[]).forEach(x=>{const o=document.createElement('option');o.value=x.index;o.textContent=x.name+' ('+x.temp_c+'C)';a.appendChild(o)});a.value=p.current_index;b.innerHTML='';(p.custom_presets||[]).forEach(x=>{const o=document.createElement('option');o.value=x.id;o.textContent=x.name+' ('+x.temp_c+'C / '+x.duration_min+'min)';b.appendChild(o)})}
async function loadCustomPresets(){const r=await jget('/api/custom-presets');el('customPresetList').textContent=(r.presets||[]).map(x=>x.id+' | '+x.name+' | '+x.target_c+'C | '+x.duration_min+'min | fan '+x.fan_base_pct+'-'+x.fan_max_pct+'% | humi '+x.humidity_stop_pct+'%').join('\n')||'暂无用户预设';await loadPresets()}
async function applyCfg(){const mode=el('modeSel').value,p={target_c:parseFloat(el('setTemp').value),duration_min:parseInt(el('setDur').value,10),idle_temp_c:parseFloat(el('setIdleTemp').value||'0'),idle_fan_pct:parseInt(el('setIdleFan').value||'0',10),fan_base_pct:parseInt(el('fanBase').value||'0',10),fan_max_pct:parseInt(el('fanMax').value||'0',10),humidity_stop_pct:parseFloat(el('humiStop').value||'0'),humidity_stable_delta_pct:parseFloat(el('humiStable').value||'0')};if(mode==='preset'){p.use_custom=false;p.preset_index=parseInt(el('presetSel').value,10)}else if(mode==='custom_preset'){p.use_custom=true;p.custom_preset_id=el('customPresetSel').value}else p.use_custom=true;await jpost('/api/control',p);showMsg('控制参数已下发');await refresh()}
async function applyPid(){await jpost('/api/control',{pid_kp:parseFloat(el('setKp').value),pid_ki:parseFloat(el('setKi').value),pid_kd:parseFloat(el('setKd').value)});showMsg('PID 已保存');await refresh()}
async function sendCtrl(cmd){await jpost('/api/control',{cmd});showMsg('命令已发送: '+cmd);await refresh()}
async function saveWifi(){await jpost('/api/wifi',{ssid:el('ssid').value,password:el('pass').value});showMsg('Wi-Fi 配置已提交','bad')}
async function saveCustomPreset(){const name=el('customName').value.trim();if(!name){showMsg('请填写用户预设名称','bad');return}const selected=el('customPresetSel').value,mode=el('modeSel').value,p={name,target_c:parseFloat(el('setTemp').value),duration_min:parseInt(el('setDur').value,10),idle_temp_c:parseFloat(el('setIdleTemp').value||'0'),idle_fan_pct:parseInt(el('setIdleFan').value||'0',10),fan_base_pct:parseInt(el('fanBase').value||'0',10),fan_max_pct:parseInt(el('fanMax').value||'0',10),humidity_stop_pct:parseFloat(el('humiStop').value||'0')};if(mode==='custom_preset'&&selected)p.id=selected;await jpost('/api/custom-presets',p);showMsg('用户预设已保存');await loadCustomPresets()}
async function deleteCustomPreset(){const id=el('customPresetSel').value;if(!id){showMsg('请先选择用户预设','bad');return}await jpost('/api/custom-presets/delete',{id});showMsg('用户预设已删除','bad');await loadCustomPresets();await refresh()}
async function applySelectedCustomPreset(){const id=el('customPresetSel').value;if(!id){showMsg('请先选择用户预设','bad');return}el('modeSel').value='custom_preset';updateModeUi();await jpost('/api/control',{use_custom:true,custom_preset_id:id});showMsg('用户预设已载入');await refresh()}
async function loadHistory(){const h=await jget('/api/history');if(!h.ok){el('historyBox').textContent='暂无历史数据';return}el('historyBox').textContent='样本行数: '+h.rows+'\n温度 min/max/avg: '+fmt(h.min_temp_c,1)+' / '+fmt(h.max_temp_c,1)+' / '+fmt(h.avg_temp_c,1)+' C\n湿度 min/max/avg: '+fmt(h.min_humi_pct,1)+' / '+fmt(h.max_humi_pct,1)+' / '+fmt(h.avg_humi_pct,1)+' %\n平均加热/风扇: '+fmt(h.avg_heater_pct,1)+'% / '+fmt(h.avg_fan_pct,1)+'%\n运行样本数: '+h.active_rows}
function downloadLog(){const t=token(),url='/api/log.csv'+(t?('?token='+encodeURIComponent(t)):'');window.open(url,'_blank')}
function renderOta(r){el('otaBox').textContent='结果: '+(r.msg||'--')+'\n当前版本: '+(lastStatus?lastStatus.ota_current_version:'--')+'\n云端版本: '+(r.version??'--')+'\n标签: '+(r.tag||'--')+'\n地址: '+(r.url||'--')}
async function otaCheck(){const r=await jget('/api/ota');renderOta(r);showMsg('OTA 查询完成')}
async function otaApply(){const r=await jpost('/api/ota',{});renderOta(r);showMsg('OTA 执行结果: '+(r.msg||'--'),r.ok?'':'bad')}
function appendDevLog(msg){const b=el('devLog');b.value+=msg+'\n';b.scrollTop=b.scrollHeight}
async function sendDevCmd(){const cmd=el('devCmd').value.trim();if(!cmd)return;appendDevLog('> '+cmd);try{const r=await jpost('/api/devcmd',{cmd});appendDevLog((r.output||'[无回显]').replace(/\n$/,''))}catch(e){appendDevLog('[错误] '+(e.message||e))}el('devCmd').value=''}
async function sendDevKey(k){appendDevLog('> btn '+k);await jpost('/api/devcmd',{cmd:'btn '+k});await refresh()}
async function applySensorType(){await jpost('/api/control',{sensor_type:el('sensorType').value});showMsg('传感器类型已更新');await refresh()}
async function applyFanPwmPol(){await jpost('/api/control',{fan_pwm_active_low:el('fanPwmPol').value==='active_low'});showMsg('PWM 极性已更新');await refresh()}
async function refresh(){const s=await jget('/api/status');lastStatus=s;setText('temp',fmt(s.temp_c,1));setText('humi',fmt(s.humi_pct,1));setText('target',fmt(s.target_c,1));setText('remain',s.remaining_hms||'--');setText('active',s.active?'运行中':'已停止');setText('preset',s.preset||'--');setText('heater',s.heater_pct??'--');setText('fanPct',s.fan_pct??'--');setText('fan',s.fan_rpm??'--');setText('fanSigV',fmt(s.fan_pwm_avg_v,2));setText('fault',(s.fault_text||'').trim()||(s.fault===0?'无故障':('故障码 '+s.fault)));el('net').innerHTML=(s.wifi_connected?'<span class="ok">已联网</span>':'<span class="bad">未联网</span>')+' IP: '+s.ip+(s.ap_mode?' | AP 模式':'');setText('mqttState',s.mqtt_connected?'已连接':(s.mqtt_key_present?'待连接':'未配置私钥'));setText('notifyState',s.notify_enabled?'已启用':'未启用');setText('otaState',s.ota_last_msg||'--');el('notifyDetail').textContent='最近通知类型: '+(s.last_notify_kind||'--')+'\n最近通知内容: '+(s.last_notify_msg||'--');setText('mqttCtlTopic',s.mqtt_control_topic||'--');setText('mqttStaTopic',s.mqtt_status_topic||'--');if(canFill('sensorType')&&s.sensor_type)el('sensorType').value=s.sensor_type;if(canFill('fanPwmPol'))el('fanPwmPol').value=s.fan_pwm_active_low?'active_low':'active_high';const mode=s.user_custom_mode?(s.active_custom_preset_id?'custom_preset':'custom'):'preset';el('modeSel').value=mode;updateModeUi();if(!uiInited||s.preset_index!==lastPresetIndex||lastCustomPresetId!==s.active_custom_preset_id){fillIfIdle('setTemp',fmt(s.target_c,1));fillIfIdle('setDur',Math.round((s.config_duration_sec||0)/60));fillIfIdle('setIdleTemp',s.idle_temp_c<35?0:fmt(s.idle_temp_c,1));fillIfIdle('setIdleFan',s.idle_fan_pct??0);fillIfIdle('fanBase',s.custom_fan_base_pct??0);fillIfIdle('fanMax',s.custom_fan_max_pct??0);fillIfIdle('humiStop',fmt(s.humidity_stop_pct||0,1));fillIfIdle('humiStable',fmt(s.humidity_stable_delta_pct||0,1));fillIfIdle('setKp',fmt(s.pid_kp,4));fillIfIdle('setKi',fmt(s.pid_ki,4));fillIfIdle('setKd',fmt(s.pid_kd,4));if(canFill('presetSel'))el('presetSel').value=s.preset_index;if(canFill('customPresetSel')&&s.active_custom_preset_id)el('customPresetSel').value=s.active_custom_preset_id}setText('tuneState',s.pid_autotune_msg||'--');setText('tuneProg',s.pid_autotune_progress??0);setText('tuneStage',mapTuneStage(s.pid_autotune_stage));setText('tuneFit',mapTuneFit(s.pid_autotune_fit_state));setText('tuneAmpN',s.pid_autotune_amp_count??0);setText('tuneTuN',s.pid_autotune_period_count??0);setText('tuneAmp',(s.pid_autotune_amp_c>0)?fmt(s.pid_autotune_amp_c,3):'--');setText('tuneTu',(s.pid_autotune_tu_s>0)?fmt(s.pid_autotune_tu_s,2):'--');setText('tuneEst',(s.pid_autotune_est_kp>0&&s.pid_autotune_est_ki>0&&s.pid_autotune_est_kd>=0)?(fmt(s.pid_autotune_est_kp,3)+' / '+fmt(s.pid_autotune_est_ki,4)+' / '+fmt(s.pid_autotune_est_kd,3)):'--');el('rawJson').textContent=JSON.stringify(s,null,2);push(hs.t,s.temp_c||0);push(hs.h,s.humi_pct||0);push(hs.f,s.fan_rpm||0);push(hs.p,s.heater_pct||0);renderCharts();renderOta({msg:s.ota_last_msg,version:s.ota_last_version,tag:s.ota_last_tag,url:s.ota_last_url});lastPresetIndex=s.preset_index;lastCustomPresetId=s.active_custom_preset_id||'';uiInited=true}
async function boot(){try{await loadSettings();await loadPresets();await loadCustomPresets();await loadHistory();await refresh();showMsg('页面已就绪')}catch(e){handleError(e)}setInterval(()=>refresh().catch(handleError),1000)}
el('devCmd').addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();sendDevCmd()}});boot();
</script></body></html>)HTML";

// 主要功能：返回 Web 首页（状态 + 控制 + 配网）。
// 使用方法：浏览器访问设备 IP 根路径触发。
static void handleWebRoot() {
  g_webServer.send_P(200, "text/html; charset=utf-8", WEB_ROOT_PAGE);
  return;
  static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>3D耗材烘干箱</title>
<style>a
:root{--bg:#eef2f7;--card:#fff;--txt:#0f172a;--sub:#64748b;--line:#d7dee7;--good:#166534;--bad:#b91c1c;--a:#0ea5e9}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font-family:"Segoe UI","PingFang SC","Microsoft YaHei",sans-serif}
.wrap{max-width:1400px;margin:0 auto;padding:14px}
.card{background:var(--card);border-radius:14px;padding:14px;box-shadow:0 3px 14px rgba(15,23,42,.08)}
.head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:12px}
.head h1{margin:0;font-size:22px}.sub{color:var(--sub);font-size:13px}
.dash{display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:12px}
.span12{grid-column:span 12}.span6{grid-column:span 6}.span4{grid-column:span 4}.span3{grid-column:span 3}
.kpi{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.kv{background:#f8fafc;border:1px solid var(--line);border-radius:10px;padding:10px}
.k{font-size:12px;color:var(--sub)}.v{font-size:20px;font-weight:700}
.panel h2{margin:0 0 10px 0;font-size:17px}.row{display:flex;gap:8px}.row button{flex:1}
label{display:block;margin:8px 0 4px;font-size:13px}input,select,button,textarea{width:100%;padding:9px;border:1px solid #cbd5e1;border-radius:8px;background:#fff}
canvas{width:100%;height:170px;background:#fff;border:1px solid var(--line);border-radius:10px}
.ok{color:var(--good)}.bad{color:var(--bad)}
.devtip{font-size:12px;color:#94a3b8;user-select:none;cursor:pointer}
textarea{font-family:Consolas,"Courier New",monospace;resize:vertical}
@media (max-width:1100px){.span6,.span4,.span3{grid-column:span 12}.kpi{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body><div class="wrap">
<div class="head"><h1 id="titleMain">3D打印耗材烘干箱</h1><div id="net" class="sub">网络状态加载中...</div></div>
<div class="dash">
  <div class="card span12">
    <div class="kpi">
      <div class="kv"><div class="k">温度</div><div class="v"><span id="temp">--</span> C</div></div>
      <div class="kv"><div class="k">湿度</div><div class="v"><span id="humi">--</span> %</div></div>
      <div class="kv"><div class="k">目标/剩余</div><div class="v"><span id="target">--</span>C / <span id="remain">--</span></div></div>
      <div class="kv"><div class="k">运行/材料</div><div class="v"><span id="active">--</span> / <span id="preset">--</span></div></div>
    </div>
    <div class="sub" style="margin-top:8px">加热 <span id="heater">--</span>% | 风扇转速(回传) <span id="fan">--</span> RPM | 风扇PWM信号 ~<span id="fanSigV">--</span>V/3.3V | <span id="fault" class="bad">无故障</span></div>
  </div>
  <div class="card span6 panel"><h2>温度曲线</h2><canvas id="cTemp" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>湿度曲线</h2><canvas id="cHumi" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>风扇转速曲线(RPM)</h2><canvas id="cFan" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>加热输出曲线(%)</h2><canvas id="cHeat" width="600" height="170"></canvas></div>
  <div class="card span4 panel"><h2>控制</h2>
    <label>参数模式</label><select id="modeSel" onchange="onModeChange()"><option value="custom">user</option><option value="preset">材料预设</option></select>
    <label>目标温度 (35~120 C)</label><input id="setTemp" type="number" step="0.5" min="35" max="120">
    <label>烘干时长 (分钟)</label><input id="setDur" type="number" min="30" max="1440">
    <label>闲时温度 (0=关闭, 35~120 C)</label><input id="setIdleTemp" type="number" step="0.5" min="0" max="120">
    <label>空闲风扇转速 (0~100 %)</label><input id="setIdleFan" type="number" min="0" max="100">
    <div id="presetWrap"><label>材料预设</label><select id="presetSel" onchange="onPresetChange()"></select></div>
    <div class="row" style="margin-top:10px"><button onclick="sendCtrl('start')">启动</button><button onclick="sendCtrl('stop')">停止</button><button onclick="sendCtrl('faultreset')">清除故障</button><button onclick="applyCfg()">应用</button></div>
  </div>
  <div class="card span4 panel"><h2>PID</h2>
    <label>Kp</label><input id="setKp" type="number" step="0.001" min="0" max="10">
    <label>Ki</label><input id="setKi" type="number" step="0.0001" min="0" max="1">
    <label>Kd</label><input id="setKd" type="number" step="0.001" min="0" max="20">
    <div class="row" style="margin-top:10px"><button onclick="applyPid()">保存PID</button><button onclick="sendCtrl('pidreset')">恢复默认</button><button onclick="sendCtrl('autotune')">自动校准</button></div>
    <div class="sub" style="margin-top:8px">状态 <span id="tuneState">IDLE</span> | 进度 <span id="tuneProg">0</span>%</div>
    <div class="sub" style="margin-top:4px">阶段 <span id="tuneStage">待机</span> | 拟合 <span id="tuneFit">未开始</span></div>
    <div class="sub" style="margin-top:4px">样本 A/Tu: <span id="tuneAmpN">0</span>/<span id="tuneTuN">0</span> | A=<span id="tuneAmp">--</span>C | Tu=<span id="tuneTu">--</span>s</div>
    <div class="sub" style="margin-top:4px">预估 Kp/Ki/Kd: <span id="tuneEst">--</span></div>
  </div>
  <div class="card span4 panel"><h2>Wi-Fi</h2>
    <label>SSID</label><input id="ssid" type="text" placeholder="路由器名称">
    <label>密码</label><input id="pass" type="password" placeholder="路由器密码">
    <button style="margin-top:10px" onclick="saveWifi()">保存并连接</button>
    <div class="sub" style="margin-top:8px">AP模式: Dryer-Config / 12345678 / 192.168.4.1</div>
  </div>
  <div id="devPanel" class="card span12 panel" style="display:none">
    <h2>开发人员选项</h2>
    <div class="sub">可模拟串口命令并查看回显，支持 help/start/stop/status/faultreset 等。</div>
    <label>温湿度传感器</label>
    <div style="display:flex;gap:8px;align-items:center">
      <select id="sensorType" style="flex:1">
        <option value="aht10">AHT10</option>
        <option value="sht3x">SHT3X</option>
      </select>
      <button style="width:180px;min-width:180px" onclick="applySensorType()">应用并重初始化</button>
    </div>
    <label>PWM极性</label>
    <div style="display:flex;gap:8px;align-items:center">
      <select id="fanPwmPol" style="flex:1">
        <option value="active_low">有效低电平(推荐)</option>
        <option value="active_high">有效高电平</option>
      </select>
      <button style="width:180px;min-width:180px" onclick="applyFanPwmPol()">应用极性</button>
    </div>
    <label>虚拟按键</label>
    <div class="row">
      <button onclick="sendDevKey('up')">上</button>
      <button onclick="sendDevKey('down')">下</button>
      <button onclick="sendDevKey('left')">左</button>
      <button onclick="sendDevKey('right')">右</button>
      <button onclick="sendDevKey('ok')">中</button>
    </div>
    <label>命令输入</label>
    <div style="display:flex;gap:8px;align-items:center">
      <input id="devCmd" type="text" placeholder="例如: help" style="flex:1">
      <button style="width:120px;min-width:120px" onclick="sendDevCmd()">发送</button>
    </div>
    <label>命令回显</label>
    <textarea id="devLog" rows="10" readonly></textarea>
    <div class="devtip" id="devHint">隐藏入口：连续点击页面标题 5 次可显示/隐藏本面板</div>
  </div>
</div>
<script>
const HIS=180,hs={t:[],h:[],f:[],p:[]};
let uiInited=false,lastPresetIndex=-1,forceParamSync=true,forcePidSync=true;
let devTapCount=0,devLastTapMs=0,devPanelShown=false;
async function jget(u){const r=await fetch(u);return await r.json();}
async function jpost(u,d){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});return await r.json();}
function setText(id,v){document.getElementById(id).textContent=v;}
function mapTuneStage(s){
  const m={IDLE:'待机',HEATUP:'升温激励',SEARCH_AMP:'振幅采样',WAIT_PERIOD:'周期等待',COLLECT:'样本积累',FIT:'拟合计算',DONE:'完成',FAIL:'失败'};
  return m[s]||s||'--';
}
function mapTuneFit(s){
  const m={N_A:'未开始',INSUFFICIENT:'样本不足',AMP_LOW:'振幅过小',PERIOD_SHORT:'周期偏短',PREVIEW:'可预估',READY:'可拟合',DONE:'拟合成功',FAIL:'拟合失败'};
  return m[s]||s||'--';
}
function appendDevLog(msg){const box=document.getElementById('devLog');if(!box)return;box.value+=msg+'\n';box.scrollTop=box.scrollHeight;}
function toggleDevPanel(forceShow){
  const panel=document.getElementById('devPanel');
  devPanelShown=(typeof forceShow==='boolean')?forceShow:(!devPanelShown);
  panel.style.display=devPanelShown?'block':'none';
  appendDevLog(devPanelShown?'[系统] 开发人员选项已开启':'[系统] 开发人员选项已隐藏');
}
function onTitleClick(){
  const now=Date.now();
  if(now-devLastTapMs>1800) devTapCount=0;
  devLastTapMs=now;
  devTapCount++;
  if(devTapCount>=5){devTapCount=0;toggleDevPanel();}
}
function push(arr,v){arr.push(v);if(arr.length>HIS)arr.shift();}
function updateModeUi(){const mode=document.getElementById('modeSel').value;document.getElementById('presetWrap').style.display=(mode==='preset')?'block':'none';}
async function onModeChange(){
  updateModeUi();
  const mode=document.getElementById('modeSel').value;
  if(mode==='preset'){
    await jpost('/api/control',{use_custom:false,preset_index:parseInt(document.getElementById('presetSel').value,10)});
  }else{
    await jpost('/api/control',{use_custom:true});
  }
  forceParamSync=true;
  await refresh();
}
async function onPresetChange(){
  const mode=document.getElementById('modeSel').value;
  if(mode!=='preset') return;
  await jpost('/api/control',{use_custom:false,preset_index:parseInt(document.getElementById('presetSel').value,10)});
  forceParamSync=true;
  await refresh();
}
function canFill(id){const el=document.getElementById(id);return document.activeElement!==el;}
function calcRange(data,opt){
  if(data.length===0){return {min:opt.defMin,max:opt.defMax};}
  let mn=Math.min(...data),mx=Math.max(...data);
  let span=Math.max(mx-mn,opt.minSpan);
  const pad=span*opt.pad;
  let lo=mn-pad,hi=mx+pad;
  if(opt.hardMin!==null) lo=Math.max(opt.hardMin,lo);
  if(opt.hardMax!==null) hi=Math.min(opt.hardMax,hi);
  if((hi-lo)<opt.minSpan){const mid=(hi+lo)/2;lo=mid-opt.minSpan/2;hi=mid+opt.minSpan/2;}
  if(opt.hardMin!==null&&lo<opt.hardMin) lo=opt.hardMin;
  if(opt.hardMax!==null&&hi>opt.hardMax) hi=opt.hardMax;
  return {min:lo,max:hi};
}
function draw(canvasId,data,color,minV,maxV){
  const c=document.getElementById(canvasId),ctx=c.getContext('2d');const w=c.width,h=c.height,p=18;
  ctx.clearRect(0,0,w,h);ctx.strokeStyle='#d7dee7';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(p,p);ctx.lineTo(p,h-p);ctx.lineTo(w-p,h-p);ctx.stroke();
  const mid=(minV+maxV)/2;ctx.fillStyle='#64748b';ctx.font='11px sans-serif';
  ctx.fillText(maxV.toFixed(1),2,p+2);ctx.fillText(mid.toFixed(1),2,h/2+3);ctx.fillText(minV.toFixed(1),2,h-p+3);
  if(data.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
  for(let i=0;i<data.length;i++){const x=p+(i*(w-2*p))/Math.max(1,HIS-1);const y=h-p-((data[i]-minV)/(maxV-minV))*(h-2*p);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}
  ctx.stroke();
}
function renderCharts(){
  const rT=calcRange(hs.t,{hardMin:null,hardMax:null,defMin:20,defMax:60,minSpan:3,pad:0.2});
  const rH=calcRange(hs.h,{hardMin:0,hardMax:100,defMin:20,defMax:80,minSpan:8,pad:0.15});
  const rF=calcRange(hs.f,{hardMin:0,hardMax:null,defMin:0,defMax:3000,minSpan:300,pad:0.15});
  const rP=calcRange(hs.p,{hardMin:0,hardMax:100,defMin:0,defMax:100,minSpan:10,pad:0.1});
  draw('cTemp',hs.t,'#ef4444',rT.min,rT.max); draw('cHumi',hs.h,'#3b82f6',rH.min,rH.max);
  draw('cFan',hs.f,'#22c55e',rF.min,rF.max); draw('cHeat',hs.p,'#f59e0b',rP.min,rP.max);
}
async function refresh(){
  const s=await jget('/api/status');
  setText('temp',s.temp_c.toFixed(1)); setText('humi',s.humi_pct.toFixed(1)); setText('target',s.target_c.toFixed(1));
  setText('remain',s.remaining_hms); setText('preset',s.preset); setText('active',s.active?'运行中':'已停止');
  setText('heater',s.heater_pct); setText('fan',s.fan_rpm);
  const fanV=(typeof s.fan_pwm_avg_v==='number')?s.fan_pwm_avg_v.toFixed(2):((s.fan_pct*3.3/100).toFixed(2));
  setText('fanSigV',fanV);
  document.getElementById('fault').textContent=s.fault===0?'无故障':'故障码: '+s.fault;
  document.getElementById('net').innerHTML=(s.wifi_connected?'<span class="ok">已联网</span> ':'<span class="bad">未联网</span> ')+'IP: '+s.ip+(s.ap_mode?' | AP配网模式':'');
  if(canFill('sensorType') && s.sensor_type){ document.getElementById('sensorType').value=s.sensor_type; }
  if(canFill('fanPwmPol')){ document.getElementById('fanPwmPol').value=s.fan_pwm_active_low?'active_low':'active_high'; }
  const presetChanged=(s.preset_index!==lastPresetIndex);
  const allowParamFill=(!uiInited)||forceParamSync||(!s.user_custom_mode&&presetChanged);
  if(allowParamFill){
    if(canFill('setTemp')) document.getElementById('setTemp').value=s.target_c.toFixed(1);
    if(canFill('setDur')) document.getElementById('setDur').value=Math.round(s.config_duration_sec/60);
    if(canFill('setIdleTemp')) document.getElementById('setIdleTemp').value=s.idle_temp_c.toFixed(1);
    if(canFill('setIdleFan')) document.getElementById('setIdleFan').value=s.idle_fan_pct;
    forceParamSync=false;
  }
  document.getElementById('modeSel').value=s.user_custom_mode?'custom':'preset'; updateModeUi();
  if((!uiInited)||forcePidSync){
    if(canFill('setKp')) document.getElementById('setKp').value=s.pid_kp.toFixed(4);
    if(canFill('setKi')) document.getElementById('setKi').value=s.pid_ki.toFixed(4);
    if(canFill('setKd')) document.getElementById('setKd').value=s.pid_kd.toFixed(4);
    forcePidSync=false;
  }
  if(canFill('presetSel')) document.getElementById('presetSel').value=s.preset_index;
  setText('tuneState',s.pid_autotune_msg); setText('tuneProg',s.pid_autotune_progress);
  setText('tuneStage',mapTuneStage(s.pid_autotune_stage));
  setText('tuneFit',mapTuneFit(s.pid_autotune_fit_state));
  setText('tuneAmpN',s.pid_autotune_amp_count??0);
  setText('tuneTuN',s.pid_autotune_period_count??0);
  const tuneAmp=(typeof s.pid_autotune_amp_c==='number'&&s.pid_autotune_amp_c>0)?s.pid_autotune_amp_c.toFixed(3):'--';
  const tuneTu=(typeof s.pid_autotune_tu_s==='number'&&s.pid_autotune_tu_s>0)?s.pid_autotune_tu_s.toFixed(2):'--';
  setText('tuneAmp',tuneAmp);
  setText('tuneTu',tuneTu);
  let est='--';
  if(typeof s.pid_autotune_est_kp==='number'&&s.pid_autotune_est_kp>0&&
     typeof s.pid_autotune_est_ki==='number'&&s.pid_autotune_est_ki>0&&
     typeof s.pid_autotune_est_kd==='number'&&s.pid_autotune_est_kd>=0){
    est=s.pid_autotune_est_kp.toFixed(3)+' / '+s.pid_autotune_est_ki.toFixed(4)+' / '+s.pid_autotune_est_kd.toFixed(3);
  }
  setText('tuneEst',est);
  push(hs.t,s.temp_c); push(hs.h,s.humi_pct); push(hs.f,s.fan_rpm); push(hs.p,s.heater_pct); renderCharts();
  lastPresetIndex=s.preset_index; uiInited=true;
}
async function loadPresets(){const p=await jget('/api/presets');const sel=document.getElementById('presetSel');sel.innerHTML='';p.presets.forEach(x=>{const o=document.createElement('option');o.value=x.index;o.textContent=x.name+' ('+x.temp_c+'C)';sel.appendChild(o);});sel.value=p.current_index;}
async function applyCfg(){const mode=document.getElementById('modeSel').value;const payload={target_c:parseFloat(document.getElementById('setTemp').value),idle_temp_c:parseFloat(document.getElementById('setIdleTemp').value),duration_min:parseInt(document.getElementById('setDur').value,10),idle_fan_pct:parseInt(document.getElementById('setIdleFan').value,10),use_custom:(mode==='custom')};if(mode==='preset'){payload.preset_index=parseInt(document.getElementById('presetSel').value,10);}await jpost('/api/control',payload);forceParamSync=true;await refresh();}
async function applyPid(){const payload={pid_kp:parseFloat(document.getElementById('setKp').value),pid_ki:parseFloat(document.getElementById('setKi').value),pid_kd:parseFloat(document.getElementById('setKd').value)};await jpost('/api/control',payload);forcePidSync=true;await refresh();}
async function sendCtrl(cmd){await jpost('/api/control',{cmd});if(cmd==='pidreset'){forcePidSync=true;}await refresh();}
async function saveWifi(){await jpost('/api/wifi',{ssid:document.getElementById('ssid').value,password:document.getElementById('pass').value});}
async function sendDevCmd(){
  const input=document.getElementById('devCmd');
  const cmd=input.value.trim();
  if(!cmd) return;
  appendDevLog('> '+cmd);
  try{
    const r=await jpost('/api/devcmd',{cmd});
    if(r&&typeof r.output==='string'){
      appendDevLog(r.output.replace(/\n$/,''));
    }else{
      appendDevLog('[系统] 无回显数据');
    }
  }catch(e){
    appendDevLog('[错误] 指令发送失败');
  }
  input.value='';
}
async function sendDevKey(k){
  await jpost('/api/devcmd',{cmd:'btn '+k});
  appendDevLog('> btn '+k);
  await refresh();
}
async function applySensorType(){
  const v=document.getElementById('sensorType').value;
  try{
    const r=await jpost('/api/control',{sensor_type:v});
    appendDevLog('[系统] 传感器已切换: '+v.toUpperCase()+(r&&r.ok===false&&r.msg?(' | '+r.msg):''));
  }catch(e){
    appendDevLog('[错误] 传感器切换失败');
  }
  await refresh();
}
async function applyFanPwmPol(){
  const pol=document.getElementById('fanPwmPol').value;
  const activeLow=(pol==='active_low');
  try{
    await jpost('/api/control',{fan_pwm_active_low:activeLow});
    appendDevLog('[系统] PWM极性已切换: '+(activeLow?'有效低电平':'有效高电平'));
  }catch(e){
    appendDevLog('[错误] PWM极性切换失败');
  }
  await refresh();
}
document.getElementById('titleMain').addEventListener('click',onTitleClick);
document.getElementById('devCmd').addEventListener('keydown',(e)=>{if(e.key==='Enter'){e.preventDefault();sendDevCmd();}});
loadPresets().then(refresh);setInterval(refresh,1000);
</script></body></html>)HTML";
  g_webServer.send(200, "text/html; charset=utf-8", PAGE);
}

// 主要功能：输出实时状态 JSON，供网页轮询刷新。
// 使用方法：前端每秒 GET /api/status。
static void fillStatusJson(JsonDocument& doc) {
  doc["temp_c"] = g_smoothTempC;
  doc["humi_pct"] = g_smoothHumi;
  doc["target_c"] = g_targetTempC;
  doc["idle_temp_c"] = g_idleTempC;
  doc["config_duration_sec"] = g_configDurationSec;
  doc["remaining_sec"] = g_remainingSec;
  doc["remaining_hms"] = formatDurationForWeb(g_remainingSec);
  doc["preset"] = currentProfileName();
  doc["preset_index"] = static_cast<uint32_t>(g_presetIndex);
  doc["active_custom_preset_id"] = g_activeCustomPresetId;
  doc["active_custom_preset_name"] = g_activeCustomPresetName;
  doc["user_custom_mode"] = g_userCustomMode;
  doc["active"] = g_dryingActive;
  doc["heater_pct"] = RatioToPct(g_heaterDemand);
  doc["fan_pct"] = g_fanPct;
  doc["fan_rpm"] = g_fanRpm;
  doc["fan_pwm_high_v"] = FAN_SIGNAL_HIGH_V;
  doc["fan_pwm_avg_v"] = (static_cast<float>(g_fanPwmDutyRaw) * FAN_SIGNAL_HIGH_V) / 255.0f;
  doc["fan_pwm_duty_raw"] = g_fanPwmDutyRaw;
  doc["fan_pwm_active_low"] = g_fanPwmActiveLow;
  doc["idle_fan_pct"] = g_idleFanPct;
  doc["custom_fan_base_pct"] = g_customFanBasePct;
  doc["custom_fan_max_pct"] = g_customFanMaxPct;
  doc["humidity_stop_pct"] = g_humidityStopPct;
  doc["humidity_stable_delta_pct"] = g_humidityStableDeltaPct;
  doc["sensor_type"] = sensorTypeToString(g_sensorType);
  doc["sensor_ok"] = g_sensorOk;
  doc["pid_kp"] = g_pidKp;
  doc["pid_ki"] = g_pidKi;
  doc["pid_kd"] = g_pidKd;
  doc["pid_autotune_msg"] = g_pidAutoTuneMsg;
  doc["fault"] = g_faultFlags;
  doc["fault_text"] = faultFlagsToText(g_faultFlags);
  doc["pid_autotune"] = g_pidAutoTuneActive;
  doc["wifi_connected"] = g_wifiConnected;
  doc["ap_mode"] = g_apConfigMode;
  doc["ip"] = g_wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["mqtt_enabled"] = g_mqttEnabled;
  doc["mqtt_connected"] = g_mqttConnected;
  doc["mqtt_key_present"] = (g_bemfaMqttKey.length() > 0);
  doc["mqtt_control_topic"] = mqttControlTopic();
  doc["mqtt_status_topic"] = mqttStatusTopic();
  doc["notify_enabled"] = g_bemfaNotificationsEnabled;
  doc["notify_group"] = g_bemfaNotifyGroup;
  doc["last_notify_kind"] = g_lastNotifyKind;
  doc["last_notify_msg"] = g_lastNotifyMsg;
  doc["ota_current_version"] = DRYER_FIRMWARE_VERSION;
  doc["ota_last_version"] = g_lastOtaVersion;
  doc["ota_last_msg"] = g_lastOtaMsg;
  doc["ota_last_tag"] = g_lastOtaTag;
  doc["ota_last_url"] = g_lastOtaUrl;
  doc["ota_auto_check"] = g_otaAutoCheckEnabled;
  doc["auth_enabled"] = (g_webToken.length() > 0);
  doc["user_preset_count"] = g_userPresetCount;

  uint8_t tuneProgress = 0;
  const char* tuneStage = "IDLE";
  const char* tuneFitState = "N_A";
  uint8_t tuneAmpCount = g_pidAutoTuneState.ampCount;
  uint8_t tunePeriodCount = g_pidAutoTuneState.periodCount;
  float tuneAmpC = (tuneAmpCount > 0) ? (g_pidAutoTuneState.ampSum / tuneAmpCount) : 0.0f;
  float tuneTuS = (tunePeriodCount > 0) ? (g_pidAutoTuneState.periodSum / tunePeriodCount) : 0.0f;
  float estKp = 0.0f;
  float estKi = 0.0f;
  float estKd = 0.0f;
  bool fitReady = false;
  uint32_t elapsedMs = 0;
  if (g_pidAutoTuneActive && g_pidAutoTuneState.startMs > 0) {
    elapsedMs = millis() - g_pidAutoTuneState.startMs;
    float pTime = AUTOTUNE_CFG.minTimeMs > 0 ? static_cast<float>(elapsedMs) / AUTOTUNE_CFG.minTimeMs : 0.0f;
    float pAmp = static_cast<float>(g_pidAutoTuneState.ampCount) / 4.0f;
    float pPeriod = static_cast<float>(g_pidAutoTuneState.periodCount) / 3.0f;
    float p = min(1.0f, min(pTime, min(pAmp, pPeriod)));
    tuneProgress = static_cast<uint8_t>(p * 100.0f);

    if (g_pidAutoTuneState.ampCount == 0 && g_pidAutoTuneState.periodCount == 0) {
      tuneStage = "HEATUP";
    } else if (g_pidAutoTuneState.ampCount < 2) {
      tuneStage = "SEARCH_AMP";
    } else if (g_pidAutoTuneState.periodCount < 1) {
      tuneStage = "WAIT_PERIOD";
    } else if (pAmp < 1.0f || pPeriod < 1.0f || pTime < 1.0f) {
      tuneStage = "COLLECT";
    } else {
      tuneStage = "FIT";
    }

    if (tuneAmpCount < 2 || tunePeriodCount < 1) {
      tuneFitState = "INSUFFICIENT";
    } else if (tuneAmpC <= AUTOTUNE_CFG.minAmplitudeC) {
      tuneFitState = "AMP_LOW";
    } else if (tuneTuS < 2.0f) {
      tuneFitState = "PERIOD_SHORT";
    } else if (tuneAmpCount < 4 || tunePeriodCount < 3 || pTime < 1.0f) {
      tuneFitState = "PREVIEW";
      fitReady = true;
    } else {
      tuneFitState = "READY";
      fitReady = true;
    }
  } else if (String(g_pidAutoTuneMsg) == "DONE") {
    tuneProgress = 100;
    tuneStage = "DONE";
    tuneFitState = "DONE";
    fitReady = true;
    estKp = g_pidKp;
    estKi = g_pidKi;
    estKd = g_pidKd;
  } else if (String(g_pidAutoTuneMsg) == "FAIL") {
    tuneStage = "FAIL";
    tuneFitState = "FAIL";
  }

  if (fitReady && tuneAmpC > AUTOTUNE_CFG.minAmplitudeC && tuneTuS > 0.0f) {
    float ku = (4.0f * AUTOTUNE_CFG.relayAmplitude) / (PI * tuneAmpC);
    float kpk = 0.6f * ku;
    float kik = (1.2f * ku) / tuneTuS;
    float kdk = 0.075f * ku * tuneTuS;
    estKp = ClampF(kpk, 0.01f, 3.0f);
    estKi = ClampF(kik, 0.0001f, 0.20f);
    estKd = ClampF(kdk, 0.0f, 20.0f);
  }
  doc["pid_autotune_progress"] = tuneProgress;
  doc["pid_autotune_stage"] = tuneStage;
  doc["pid_autotune_fit_state"] = tuneFitState;
  doc["pid_autotune_amp_count"] = tuneAmpCount;
  doc["pid_autotune_period_count"] = tunePeriodCount;
  doc["pid_autotune_amp_c"] = tuneAmpC;
  doc["pid_autotune_tu_s"] = tuneTuS;
  doc["pid_autotune_est_kp"] = estKp;
  doc["pid_autotune_est_ki"] = estKi;
  doc["pid_autotune_est_kd"] = estKd;
  doc["pid_autotune_fit_ready"] = fitReady;
  doc["pid_autotune_elapsed_ms"] = elapsedMs;
}

static bool applyControlJson(JsonDocument& in, String& errMsg) {
  if (in["cmd"].is<const char*>()) {
    String cmd = in["cmd"].as<String>();
    cmd.toLowerCase();
    if (cmd == "start") {
      startDrying();
    } else if (cmd == "stop") {
      stopDrying();
    } else if (cmd == "faultreset") {
      clearFaults();
      saveState(true);
    } else if (cmd == "autotune") {
      startPidAutoTune();
    } else if (cmd == "pidreset") {
      g_pidKp = PID_DEFAULT_KP;
      g_pidKi = PID_DEFAULT_KI;
      g_pidKd = PID_DEFAULT_KD;
      g_pidInited = false;
      snprintf(g_pidAutoTuneMsg, sizeof(g_pidAutoTuneMsg), "IDLE");
      saveState(true);
    } else if (cmd == "ota_check") {
      return checkBemfaOta(false, errMsg);
    } else if (cmd == "ota_apply") {
      return checkBemfaOta(true, errMsg);
    }
  }

  if (in["sensor_type"].is<const char*>()) {
    String sensorType = in["sensor_type"].as<String>();
    sensorType.toLowerCase();
    if (sensorType == "aht10") {
      g_sensorType = SENSOR_TYPE_AHT10;
    } else if (sensorType == "sht3x" || sensorType == "sht31" || sensorType == "sht30") {
      g_sensorType = SENSOR_TYPE_SHT3X;
    } else {
      errMsg = "sensor_type invalid";
      return false;
    }
    initTempHumiditySensor();
  }

  if (in["fan_pwm_active_low"].is<bool>()) {
    g_fanPwmActiveLow = in["fan_pwm_active_low"].as<bool>();
    setFanPct(g_fanPct);
  }

  if (in["custom_preset_id"].is<const char*>()) {
    if (!applyUserPresetById(in["custom_preset_id"].as<String>())) {
      errMsg = "custom preset not found";
      return false;
    }
  }

  bool hasTarget = (in["target_c"].is<float>() || in["target_c"].is<int>());
  bool hasDuration = in["duration_min"].is<int>();
  bool hasPreset = in["preset_index"].is<int>();
  bool hasMode = in["use_custom"].is<bool>();
  bool useCustom = hasMode ? in["use_custom"].as<bool>() : false;
  bool hasManualCustom = in["fan_base_pct"].is<int>() || in["fan_max_pct"].is<int>() ||
                         in["humidity_stop_pct"].is<float>() || in["humidity_stop_pct"].is<int>();

  if (hasMode) {
    g_userCustomMode = useCustom;
    if (!useCustom) clearActiveCustomPreset();
    if (useCustom) hasPreset = false;
  }

  if (hasPreset && !g_userCustomMode) {
    int idx = in["preset_index"].as<int>();
    if (idx >= 0 && static_cast<size_t>(idx) < PRESET_COUNT) {
      applyPreset(static_cast<size_t>(idx));
      clearActiveCustomPreset();
      g_customFanBasePct = activePreset().fanBasePct;
      g_customFanMaxPct = activePreset().fanMaxPct;
      if (g_dryingActive) g_remainingSec = g_configDurationSec;
    }
  }

  if (hasTarget) {
    g_targetTempC = ClampF(in["target_c"].as<float>(), TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  }
  if (in["idle_temp_c"].is<float>() || in["idle_temp_c"].is<int>()) {
    float t = in["idle_temp_c"].as<float>();
    g_idleTempC = (t < TARGET_TEMP_MIN_C) ? IDLE_TEMP_DISABLED_C : ClampF(t, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  }
  if (hasDuration) {
    int durMin = in["duration_min"].as<int>();
    uint32_t sec = constrain(static_cast<uint32_t>(durMin) * 60UL, DURATION_MIN_SEC, DURATION_MAX_SEC);
    g_configDurationSec = sec;
    if (g_dryingActive) g_remainingSec = g_configDurationSec;
  }
  if (in["idle_fan_pct"].is<int>()) {
    g_idleFanPct = constrain(in["idle_fan_pct"].as<int>(), 0, 100);
  }
  if (in["fan_base_pct"].is<int>()) {
    g_customFanBasePct = constrain(in["fan_base_pct"].as<int>(), 0, 100);
    g_userCustomMode = true;
  }
  if (in["fan_max_pct"].is<int>()) {
    g_customFanMaxPct = constrain(in["fan_max_pct"].as<int>(), 0, 100);
    if (g_customFanMaxPct < g_customFanBasePct) g_customFanMaxPct = g_customFanBasePct;
    g_userCustomMode = true;
  }
  if (in["humidity_stop_pct"].is<float>() || in["humidity_stop_pct"].is<int>()) {
    g_humidityStopPct = ClampF(in["humidity_stop_pct"].as<float>(), 0.0f, 100.0f);
    g_userCustomMode = true;
  }
  if (in["humidity_stable_delta_pct"].is<float>() || in["humidity_stable_delta_pct"].is<int>()) {
    g_humidityStableDeltaPct = ClampF(in["humidity_stable_delta_pct"].as<float>(), 0.0f, 10.0f);
  }
  if (!hasMode && !hasPreset && (hasTarget || hasDuration || hasManualCustom)) {
    g_userCustomMode = true;
    if (!in["custom_preset_id"].is<const char*>()) clearActiveCustomPreset();
  }
  if (in["pid_kp"].is<float>() || in["pid_kp"].is<int>()) {
    g_pidKp = ClampF(in["pid_kp"].as<float>(), 0.0f, 10.0f);
    g_pidInited = false;
  }
  if (in["pid_ki"].is<float>() || in["pid_ki"].is<int>()) {
    g_pidKi = ClampF(in["pid_ki"].as<float>(), 0.0f, 1.0f);
    g_pidInited = false;
  }
  if (in["pid_kd"].is<float>() || in["pid_kd"].is<int>()) {
    g_pidKd = ClampF(in["pid_kd"].as<float>(), 0.0f, 20.0f);
    g_pidInited = false;
  }

  saveState(true);
  markStatusDirty();
  return true;
}

static bool sendBemfaNotification(const char* kind, const String& message) {
  if (!g_bemfaNotificationsEnabled || !g_wifiConnected || g_bemfaUid.length() == 0 || message.length() == 0) {
    return false;
  }
  String base = (strcmp(kind, "fault") == 0)
                    ? "http://apis.bemfa.com/vb/wechat/v1/wechatAlert"
                    : "http://apis.bemfa.com/vb/wechat/v1/wechatWarn";
  String url = base + "?uid=" + urlEncode(g_bemfaUid) + "&device=" + urlEncode("FilamentDryer") +
               "&message=" + urlEncode(message) + "&group=" + urlEncode(g_bemfaNotifyGroup);
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  http.end();
  Serial.printf("[BEMFA] notify kind=%s code=%d msg=%s\n", kind, code, message.c_str());
  snprintf(g_lastNotifyKind, sizeof(g_lastNotifyKind), "%s", kind);
  snprintf(g_lastNotifyMsg, sizeof(g_lastNotifyMsg), "%s", message.c_str());
  return code > 0 && code < 400;
}

static void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  String body;
  body.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) body += static_cast<char>(payload[i]);
  body.trim();
  if (body.length() == 0) return;
  Serial.printf("[MQTT] recv topic=%s payload=%s\n", topic ? topic : "(null)", body.c_str());

  String errMsg;
  JsonDocument in;
  if (body.startsWith("{")) {
    if (deserializeJson(in, body) == DeserializationError::Ok) {
      applyControlJson(in, errMsg);
    }
  } else {
    String lower = body;
    lower.toLowerCase();
    if (lower == "on" || lower == "start") {
      in["cmd"] = "start";
    } else if (lower == "off" || lower == "stop") {
      in["cmd"] = "stop";
    } else if (lower == "faultreset") {
      in["cmd"] = "faultreset";
    } else if (lower == "autotune") {
      in["cmd"] = "autotune";
    } else if (lower == "status") {
      publishStatusMqtt(true);
      return;
    } else {
      executeConsoleCommand(lower);
      markStatusDirty();
      return;
    }
    applyControlJson(in, errMsg);
  }
}

static void publishStatusMqtt(bool force) {
  if (!g_mqttClient.connected()) return;
  uint32_t now = millis();
  if (!force && !g_statusPublishDirty && (now - g_lastMqttPublishMs) < MQTT_PUBLISH_INTERVAL_MS) return;
  JsonDocument doc;
  fillStatusJson(doc);
  String out;
  serializeJson(doc, out);
  String topic = mqttStatusPushTopic();
  if (topic.length() == 0) return;
  bool ok = g_mqttClient.publish(topic.c_str(), out.c_str(), true);
  String preview = out;
  if (preview.length() > 180) preview = preview.substring(0, 180) + "...";
  Serial.printf("[MQTT] publish %s topic=%s bytes=%u payload=%s\n", ok ? "ok" : "fail", topic.c_str(),
                static_cast<unsigned>(out.length()), preview.c_str());
  g_lastMqttPublishMs = now;
  g_statusPublishDirty = false;
}

static void serviceMqtt() {
  g_mqttEnabled = g_wifiConnected && g_bemfaMqttKey.length() > 0 && isBemfaTopicBaseValid(g_bemfaControlTopic) &&
                  isBemfaTopicBaseValid(g_bemfaStatusTopic);
  g_mqttConnected = g_mqttClient.connected();
  if (!g_mqttEnabled) {
    if (g_lastMqttEnabledDebug || g_lastMqttConnectedDebug) {
      printBemfaMqttDisabledReason();
    }
    if (g_mqttClient.connected()) g_mqttClient.disconnect();
    g_mqttConnected = false;
    g_lastMqttEnabledDebug = false;
    g_lastMqttConnectedDebug = false;
    g_lastMqttClientStateDebug = g_mqttClient.state();
    return;
  }

  if (!g_lastMqttEnabledDebug) {
    printBemfaMqttConfig("enabled");
  }
  g_mqttClient.setServer(BEMFA_HOST, BEMFA_MQTT_PORT);
  g_mqttClient.setCallback(mqttCallback);

  if (!g_mqttClient.connected()) {
    if (g_lastMqttConnectedDebug) {
      Serial.printf("[MQTT] disconnected, state=%d\n", g_mqttClient.state());
    }
    uint32_t now = millis();
    if (now - g_lastMqttReconnectMs >= MQTT_RECONNECT_INTERVAL_MS) {
      g_lastMqttReconnectMs = now;
      Serial.printf("[MQTT] connecting host=%s:%u clientId=%s sub=%s pub=%s\n", BEMFA_HOST,
                    static_cast<unsigned>(BEMFA_MQTT_PORT), maskSecret(g_bemfaMqttKey).c_str(),
                    mqttControlTopic().c_str(), mqttStatusPushTopic().c_str());
      if (g_mqttClient.connect(g_bemfaMqttKey.c_str(), "", "")) {
        bool subOk = g_mqttClient.subscribe(mqttControlTopic().c_str());
        Serial.printf("[MQTT] connected | subscribe=%s topic=%s\n", subOk ? "ok" : "fail", mqttControlTopic().c_str());
        g_statusPublishDirty = true;
      } else {
        int8_t state = g_mqttClient.state();
        Serial.printf("[MQTT] connect failed, state=%d\n", state);
        g_lastMqttClientStateDebug = state;
      }
    }
    g_mqttConnected = g_mqttClient.connected();
    g_lastMqttEnabledDebug = true;
    g_lastMqttConnectedDebug = g_mqttConnected;
    return;
  }

  g_mqttClient.loop();
  g_mqttConnected = g_mqttClient.connected();
  if (!g_lastMqttConnectedDebug && g_mqttConnected) {
    Serial.printf("[MQTT] online | sub=%s pub=%s\n", mqttControlTopic().c_str(), mqttStatusPushTopic().c_str());
  }
  g_lastMqttEnabledDebug = true;
  g_lastMqttConnectedDebug = g_mqttConnected;
  publishStatusMqtt(false);
}

static bool checkBemfaOta(bool applyUpdate, String& resultMsg) {
  if (!g_wifiConnected || g_bemfaUid.length() == 0) {
    resultMsg = "wifi/uid required";
    Serial.printf("[BEMFA] ota skipped | wifi=%s uid=%s\n", g_wifiConnected ? "ok" : "no",
                  g_bemfaUid.length() > 0 ? "ok" : "empty");
    return false;
  }
  String topic = (g_bemfaOtaTopic.length() > 0) ? g_bemfaOtaTopic : g_bemfaControlTopic;
  if (topic.length() == 0) {
    resultMsg = "ota topic required";
    return false;
  }

  String url = "http://apis.bemfa.com/vb/api/v1/firmwareVersion?openID=" + urlEncode(g_bemfaUid) +
               "&topic=" + urlEncode(topic) + "&deviceType=1";
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  Serial.printf("[BEMFA] ota query topic=%s code=%d\n", topic.c_str(), code);
  if (code <= 0) {
    http.end();
    resultMsg = "ota query failed";
    snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
    return false;
  }
  String body = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok || (doc["code"].as<int>() != 0)) {
    resultMsg = "ota response invalid";
    snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
    return false;
  }

  JsonObject data = doc["data"].as<JsonObject>();
  uint32_t version = data["version"] | 0;
  String binUrl = data["url"] | "";
  String tag = data["tag"] | "";
  g_lastOtaVersion = version;
  g_lastOtaUrl = binUrl;
  g_lastOtaTag = tag;

  if (version <= DRYER_FIRMWARE_VERSION || binUrl.length() == 0) {
    resultMsg = "no update";
    snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
    return true;
  }

  if (!applyUpdate) {
    resultMsg = "update available";
    snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
    return true;
  }

  WiFiClient otaClient;
  t_httpUpdate_return ret = httpUpdate.update(otaClient, binUrl);
  if (ret == HTTP_UPDATE_OK) {
    resultMsg = "ota success";
    snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
    sendBemfaNotification("warn", "OTA升级成功，设备即将重启");
    return true;
  }
  if (ret == HTTP_UPDATE_NO_UPDATES) {
    resultMsg = "ota no updates";
  } else {
    resultMsg = String("ota failed: ") + httpUpdate.getLastErrorString();
  }
  snprintf(g_lastOtaMsg, sizeof(g_lastOtaMsg), "%s", resultMsg.c_str());
  sendBemfaNotification("fault", resultMsg);
  return (ret == HTTP_UPDATE_NO_UPDATES);
}

static void trackHumidityHistory(uint32_t nowMs) {
  if (!g_sensorOk || (nowMs - g_lastHumiditySampleMs) < HUMIDITY_SAMPLE_INTERVAL_MS) return;
  g_lastHumiditySampleMs = nowMs;
  g_humidityHistoryValues[g_humidityHistoryHead] = g_smoothHumi;
  g_humidityHistoryTimes[g_humidityHistoryHead] = nowMs;
  g_humidityHistoryHead = (g_humidityHistoryHead + 1) % HUMIDITY_HISTORY_CAPACITY;
  if (g_humidityHistoryCount < HUMIDITY_HISTORY_CAPACITY) g_humidityHistoryCount++;
}

static void updateDryingCompletionByHumidity() {
  if (!g_dryingActive || !g_sensorOk) return;
  uint32_t now = millis();
  bool finished = false;
  String reason;

  if (g_humidityStopPct > 0.0f && g_smoothHumi <= g_humidityStopPct) {
    finished = true;
    reason = String("湿度达到阈值：") + String(g_smoothHumi, 1) + "%";
  }

  if (!finished && g_humidityStableDeltaPct > 0.0f && g_dryingStartMs > 0 &&
      (now - g_dryingStartMs) >= HUMIDITY_STABLE_WINDOW_MS && g_humidityHistoryCount >= 2) {
    float oldest = g_smoothHumi;
    bool found = false;
    for (uint8_t i = 0; i < g_humidityHistoryCount; ++i) {
      uint8_t idx = (g_humidityHistoryHead + HUMIDITY_HISTORY_CAPACITY - 1 - i) % HUMIDITY_HISTORY_CAPACITY;
      uint32_t ts = g_humidityHistoryTimes[idx];
      if (ts == 0) continue;
      if ((now - ts) >= HUMIDITY_STABLE_WINDOW_MS) {
        oldest = g_humidityHistoryValues[idx];
        found = true;
        break;
      }
    }
    if (found && fabsf(oldest - g_smoothHumi) <= g_humidityStableDeltaPct) {
      finished = true;
      reason = String("湿度趋稳：Δ") + String(fabsf(oldest - g_smoothHumi), 2) + "%";
    }
  }

  if (finished) {
    stopDrying();
    sendBemfaNotification("warn", String("烘干完成，") + reason);
  }
}

static void updateAdvancedFaults(uint32_t nowMs) {
  if (nowMs < g_dryingStartMs) return;
  if (!g_dryingActive || !g_sensorOk || g_pidAutoTuneActive) {
    g_lastReachedSetpointMs = 0;
    g_heaterDemandZeroSinceMs = 0;
    return;
  }

  if (g_smoothTempC >= (g_targetTempC - HEAT_TIMEOUT_MARGIN_C)) {
    g_lastReachedSetpointMs = nowMs;
  } else if (g_dryingStartMs > 0 && g_lastReachedSetpointMs == 0 &&
             (nowMs - g_dryingStartMs) > HEAT_TIMEOUT_MS) {
    setFault(FAULT_HEAT_TIMEOUT, "升温超时");
    sendBemfaNotification("fault", "故障：升温超时");
    return;
  }

  if (g_heaterDemand <= 0.02f && g_smoothTempC > (g_targetTempC + HEATER_STUCK_OVER_TARGET_C)) {
    if (g_heaterDemandZeroSinceMs == 0) g_heaterDemandZeroSinceMs = nowMs;
    if ((nowMs - g_heaterDemandZeroSinceMs) > HEATER_STUCK_CONFIRM_MS) {
      setFault(FAULT_HEATER_STUCK, "疑似加热器粘连");
      sendBemfaNotification("fault", "故障：疑似加热器粘连");
    }
  } else {
    g_heaterDemandZeroSinceMs = 0;
  }
}

static void handleApiStatus() {
  if (!checkWebToken(false)) return;
  JsonDocument doc;
  fillStatusJson(doc);
  String out;
  serializeJson(doc, out);
  g_webServer.send(200, "application/json; charset=utf-8", out);
}

// 主要功能：输出预设列表 JSON，供网页下拉框选择。
// 使用方法：前端进入页面时 GET /api/presets。
static void handleApiPresets() {
  if (!checkWebToken(false)) return;
  JsonDocument doc;
  doc["current_index"] = static_cast<uint32_t>(g_presetIndex);
  JsonArray arr = doc["presets"].to<JsonArray>();
  for (size_t i = 0; i < PRESET_COUNT; ++i) {
    JsonObject p = arr.add<JsonObject>();
    p["index"] = static_cast<uint32_t>(i);
    p["name"] = PRESETS[i].name;
    p["temp_c"] = PRESETS[i].targetTempC;
  }
  JsonArray customArr = doc["custom_presets"].to<JsonArray>();
  for (uint8_t i = 0; i < g_userPresetCount; ++i) {
    const UserPreset& p = g_userPresets[i];
    if (!p.used) continue;
    JsonObject item = customArr.add<JsonObject>();
    item["id"] = p.id;
    item["name"] = p.name;
    item["temp_c"] = p.targetTempC;
    item["duration_min"] = p.durationSec / 60UL;
    item["fan_base_pct"] = p.fanBasePct;
    item["fan_max_pct"] = p.fanMaxPct;
    item["idle_temp_c"] = p.idleTempC;
    item["idle_fan_pct"] = p.idleFanPct;
    item["humidity_stop_pct"] = p.humidityStopPct;
  }

  String out;
  serializeJson(doc, out);
  g_webServer.send(200, "application/json; charset=utf-8", out);
}

// 主要功能：处理网页控制命令（启停、目标温度、时长、预设等）。
// 使用方法：POST /api/control，JSON 字段支持 cmd/target_c/duration_min/preset_index。
static void handleApiControl() {
  if (!checkWebToken(false)) return;
  JsonDocument payload;
  DeserializationError err = deserializeJson(payload, g_webServer.arg("plain"));
  if (err) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"json format error\"}");
    return;
  }
  String errMsg;
  if (!applyControlJson(payload, errMsg)) {
    String out = String("{\"ok\":false,\"msg\":\"") + errMsg + "\"}";
    g_webServer.send(400, "application/json", out);
    return;
  }
  JsonDocument out;
  out["ok"] = true;
  out["msg"] = errMsg;
  String resp;
  serializeJson(out, resp);
  g_webServer.send(200, "application/json", resp);
}

// 主要功能：处理 Wi-Fi 凭据写入与重连。
// 使用方法：POST /api/wifi，JSON 字段 ssid/password。
static void handleApiWifi() {
  if (!checkWebToken(false)) return;
  JsonDocument in;
  DeserializationError err = deserializeJson(in, g_webServer.arg("plain"));
  if (err || !in["ssid"].is<const char*>()) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"ssid不能为空\"}");
    return;
  }

  String ssid = in["ssid"].as<String>();
  String pass = in["password"].is<const char*>() ? in["password"].as<String>() : "";
  ssid.trim();
  if (ssid.length() == 0) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"ssid不能为空\"}");
    return;
  }

  saveWifiConfig(ssid, pass);
  WiFi.softAPdisconnect(true);
  g_dnsServer.stop();
  g_apConfigMode = false;

  bool ok = connectWifiSta();
  if (!ok) {
    startConfigApPortal();
    g_webServer.send(200, "application/json", "{\"ok\":false,\"msg\":\"连接失败，已回到AP配网模式\"}");
    return;
  }
  g_webServer.send(200, "application/json", "{\"ok\":true,\"msg\":\"WiFi连接成功\"}");
}

// 主要功能：开发人员调试接口，模拟串口命令并返回回显。
// 使用方法：POST /api/devcmd，JSON 字段 cmd（如 help/start/status）。
static void handleApiDevcmd() {
  if (!checkWebToken(false)) return;
  JsonDocument in;
  DeserializationError err = deserializeJson(in, g_webServer.arg("plain"));
  if (err || !in["cmd"].is<const char*>()) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"cmd不能为空\"}");
    return;
  }

  String cmd = in["cmd"].as<String>();
  cmd.trim();
  if (cmd.length() == 0) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"cmd不能为空\"}");
    return;
  }

  String outText = executeConsoleCommand(cmd);
  JsonDocument out;
  out["ok"] = true;
  out["echo"] = cmd;
  out["output"] = outText;
  String resp;
  serializeJson(out, resp);
  g_webServer.send(200, "application/json; charset=utf-8", resp);
}

// 主要功能：404 兜底处理，在 AP 模式下引导到首页。
// 使用方法：WebServer 未命中路由时自动调用。
static void handleApiSettings() {
  if (g_webServer.method() == HTTP_GET) {
    bool authed = checkWebToken(true);
    JsonDocument doc;
    doc["auth_enabled"] = (g_webToken.length() > 0);
    if (authed || g_webToken.length() == 0) {
      doc["mqtt_uid"] = g_bemfaUid;
      doc["mqtt_key"] = g_bemfaMqttKey;
      doc["mqtt_control_topic_base"] = g_bemfaControlTopic;
      doc["mqtt_status_topic_base"] = g_bemfaStatusTopic;
      doc["ota_topic_base"] = g_bemfaOtaTopic;
      doc["notify_enabled"] = g_bemfaNotificationsEnabled;
      doc["notify_group"] = g_bemfaNotifyGroup;
      doc["ota_auto_check"] = g_otaAutoCheckEnabled;
      doc["token_present"] = (g_webToken.length() > 0);
    }
    String out;
    serializeJson(doc, out);
    g_webServer.send(200, "application/json; charset=utf-8", out);
    return;
  }

  bool hadToken = (g_webToken.length() > 0);
  if (hadToken && !checkWebToken(false)) return;
  JsonDocument in;
  if (deserializeJson(in, g_webServer.arg("plain")) != DeserializationError::Ok) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"json format error\"}");
    return;
  }
  if (in["token"].is<const char*>()) {
    g_webToken = in["token"].as<String>();
    g_webToken.trim();
    prefs.putString("webtoken", g_webToken);
  }
  if (in["mqtt_uid"].is<const char*>()) {
    g_bemfaUid = in["mqtt_uid"].as<String>();
    g_bemfaUid.trim();
    prefs.putString("bemfa_uid", g_bemfaUid);
  }
  if (in["mqtt_key"].is<const char*>()) {
    g_bemfaMqttKey = in["mqtt_key"].as<String>();
    g_bemfaMqttKey.trim();
    prefs.putString("bemfa_mkey", g_bemfaMqttKey);
  }
  if (in["mqtt_control_topic_base"].is<const char*>()) {
    String topic = in["mqtt_control_topic_base"].as<String>();
    topic.trim();
    if (!isBemfaTopicBaseValid(topic)) {
      g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"mqtt control topic must be alphanumeric\"}");
      return;
    }
    g_bemfaControlTopic = topic;
    prefs.putString("bemfa_ctl", g_bemfaControlTopic);
  }
  if (in["mqtt_status_topic_base"].is<const char*>()) {
    String topic = in["mqtt_status_topic_base"].as<String>();
    topic.trim();
    if (!isBemfaTopicBaseValid(topic)) {
      g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"mqtt status topic must be alphanumeric\"}");
      return;
    }
    g_bemfaStatusTopic = topic;
    prefs.putString("bemfa_sta", g_bemfaStatusTopic);
  }
  if (in["ota_topic_base"].is<const char*>()) {
    String topic = in["ota_topic_base"].as<String>();
    topic.trim();
    if (topic.length() > 0 && !isBemfaTopicBaseValid(topic)) {
      g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"ota topic must be alphanumeric\"}");
      return;
    }
    g_bemfaOtaTopic = topic;
    prefs.putString("bemfa_ota", g_bemfaOtaTopic);
  }
  if (in["notify_enabled"].is<bool>()) {
    g_bemfaNotificationsEnabled = in["notify_enabled"].as<bool>();
    prefs.putBool("notify_en", g_bemfaNotificationsEnabled);
  }
  if (in["notify_group"].is<const char*>()) {
    g_bemfaNotifyGroup = in["notify_group"].as<String>();
    g_bemfaNotifyGroup.trim();
    if (g_bemfaNotifyGroup.length() == 0) g_bemfaNotifyGroup = "default";
    prefs.putString("notify_grp", g_bemfaNotifyGroup);
  }
  if (in["ota_auto_check"].is<bool>()) {
    g_otaAutoCheckEnabled = in["ota_auto_check"].as<bool>();
    prefs.putBool("ota_auto", g_otaAutoCheckEnabled);
  }
  if (in["mqtt_uid"].is<const char*>() || in["mqtt_key"].is<const char*>() || in["mqtt_control_topic_base"].is<const char*>() ||
      in["mqtt_status_topic_base"].is<const char*>() || in["ota_topic_base"].is<const char*>()) {
    printBemfaMqttConfig("settings updated");
  }
  markStatusDirty();
  g_webServer.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiCustomPresets() {
  if (!checkWebToken(false)) return;
  if (g_webServer.method() == HTTP_GET) {
    JsonDocument doc;
    JsonArray arr = doc["presets"].to<JsonArray>();
    for (uint8_t i = 0; i < g_userPresetCount; ++i) {
      const UserPreset& p = g_userPresets[i];
      if (!p.used) continue;
      JsonObject item = arr.add<JsonObject>();
      item["id"] = p.id;
      item["name"] = p.name;
      item["target_c"] = p.targetTempC;
      item["duration_min"] = p.durationSec / 60UL;
      item["fan_base_pct"] = p.fanBasePct;
      item["fan_max_pct"] = p.fanMaxPct;
      item["idle_temp_c"] = p.idleTempC;
      item["idle_fan_pct"] = p.idleFanPct;
      item["humidity_stop_pct"] = p.humidityStopPct;
    }
    String out;
    serializeJson(doc, out);
    g_webServer.send(200, "application/json; charset=utf-8", out);
    return;
  }
  JsonDocument in;
  if (deserializeJson(in, g_webServer.arg("plain")) != DeserializationError::Ok) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"json format error\"}");
    return;
  }
  String errMsg;
  if (!upsertUserPresetFromJson(in, errMsg)) {
    String out = String("{\"ok\":false,\"msg\":\"") + errMsg + "\"}";
    g_webServer.send(400, "application/json", out);
    return;
  }
  g_webServer.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiCustomPresetsDelete() {
  if (!checkWebToken(false)) return;
  JsonDocument in;
  if (deserializeJson(in, g_webServer.arg("plain")) != DeserializationError::Ok || !in["id"].is<const char*>()) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"id required\"}");
    return;
  }
  if (!deleteUserPresetById(in["id"].as<String>())) {
    g_webServer.send(404, "application/json", "{\"ok\":false,\"msg\":\"preset not found\"}");
    return;
  }
  g_webServer.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiHistory() {
  if (!checkWebToken(false)) return;
  HistorySummary summary;
  JsonDocument doc;
  if (!buildHistorySummary(summary)) {
    doc["ok"] = false;
    doc["msg"] = "no data";
  } else {
    doc["ok"] = true;
    doc["rows"] = summary.rows;
    doc["min_temp_c"] = summary.minTempC;
    doc["max_temp_c"] = summary.maxTempC;
    doc["avg_temp_c"] = summary.avgTempC;
    doc["min_humi_pct"] = summary.minHumiPct;
    doc["max_humi_pct"] = summary.maxHumiPct;
    doc["avg_humi_pct"] = summary.avgHumiPct;
    doc["avg_heater_pct"] = summary.avgHeaterPct;
    doc["avg_fan_pct"] = summary.avgFanPct;
    doc["active_rows"] = summary.activeRows;
  }
  String out;
  serializeJson(doc, out);
  g_webServer.send(200, "application/json; charset=utf-8", out);
}

static void handleApiLogDownload() {
  if (!checkWebToken(false)) return;
  File f = SPIFFS.open("/dryer_log.csv", FILE_READ);
  if (!f) {
    g_webServer.send(404, "text/plain", "log not found");
    return;
  }
  g_webServer.streamFile(f, "text/csv; charset=utf-8");
  f.close();
}

static void handleApiOta() {
  if (!checkWebToken(false)) return;
  String result;
  bool ok = checkBemfaOta(g_webServer.method() == HTTP_POST, result);
  JsonDocument doc;
  doc["ok"] = ok;
  doc["msg"] = result;
  doc["version"] = g_lastOtaVersion;
  doc["tag"] = g_lastOtaTag;
  doc["url"] = g_lastOtaUrl;
  String out;
  serializeJson(doc, out);
  g_webServer.send(ok ? 200 : 400, "application/json; charset=utf-8", out);
}

static void handleNotFound() {
  if (g_apConfigMode) {
    g_webServer.sendHeader("Location", "/", true);
    g_webServer.send(302, "text/plain", "");
  } else {
    g_webServer.send(404, "text/plain; charset=utf-8", "Not Found");
  }
}

// 主要功能：初始化 HTTP 路由并启动服务器。
// 使用方法：setup 中调用一次。
static void startWebServer() {
  const char* headerKeys[] = {"X-Dryer-Token"};
  g_webServer.collectHeaders(headerKeys, 1);
  g_webServer.on("/", HTTP_GET, handleWebRoot);
  g_webServer.on("/api/status", HTTP_GET, handleApiStatus);
  g_webServer.on("/api/presets", HTTP_GET, handleApiPresets);
  g_webServer.on("/api/control", HTTP_POST, handleApiControl);
  g_webServer.on("/api/wifi", HTTP_POST, handleApiWifi);
  g_webServer.on("/api/devcmd", HTTP_POST, handleApiDevcmd);
  g_webServer.on("/api/settings", HTTP_GET, handleApiSettings);
  g_webServer.on("/api/settings", HTTP_POST, handleApiSettings);
  g_webServer.on("/api/custom-presets", HTTP_GET, handleApiCustomPresets);
  g_webServer.on("/api/custom-presets", HTTP_POST, handleApiCustomPresets);
  g_webServer.on("/api/custom-presets/delete", HTTP_POST, handleApiCustomPresetsDelete);
  g_webServer.on("/api/history", HTTP_GET, handleApiHistory);
  g_webServer.on("/api/log.csv", HTTP_GET, handleApiLogDownload);
  g_webServer.on("/api/ota", HTTP_GET, handleApiOta);
  g_webServer.on("/api/ota", HTTP_POST, handleApiOta);
  g_webServer.onNotFound(handleNotFound);
  g_webServer.begin();
  Serial.println("Web 服务器已启动（80端口）。");
}

// 主要功能：维护网络服务轮询（DNS 劫持 + HTTP）。
// 使用方法：loop 中高频调用。
static void serviceNetwork() {
  static uint32_t lastReconnectMs = 0;
  static uint32_t offlineSinceMs = 0;
  g_wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (g_wifiConnected) {
    offlineSinceMs = 0;
  } else if (offlineSinceMs == 0) {
    offlineSinceMs = millis();
  }

  if (!g_wifiConnected && !g_apConfigMode && g_wifiSsid.length() == 0) {
    startConfigApPortal();
  }

  if (!g_wifiConnected && !g_apConfigMode && g_wifiSsid.length() > 0) {
    uint32_t now = millis();
    if (now - lastReconnectMs >= 10000) {
      lastReconnectMs = now;
      WiFi.disconnect();
      WiFi.begin(g_wifiSsid.c_str(), g_wifiPass.c_str());
    }
    if (offlineSinceMs != 0 && (now - offlineSinceMs) > 30000) {
      startConfigApPortal();
    }
  }

  if (g_apConfigMode) {
    g_dnsServer.processNextRequest();
  }
  g_webServer.handleClient();
  serviceMqtt();
  if (g_otaAutoCheckEnabled && g_wifiConnected && (millis() - g_lastOtaCheckMs) > OTA_AUTO_CHECK_INTERVAL_MS) {
    String msg;
    g_lastOtaCheckMs = millis();
    checkBemfaOta(false, msg);
  }
}

// 主要功能：控制加热 MOSFET 开关。
// 使用方法：on=true 打开加热，false 关闭加热。
static void setHeater(bool on) {
  g_heaterOn = on;
  digitalWrite(HEATER_PIN, on ? HIGH : LOW);
}

// 主要功能：设置风扇 PWM 占空比百分比。
// 使用方法：传入 0~100，内部自动限幅并写入 LEDC。
static void setFanPct(uint8_t pct) {
  g_fanPct = pct > 100 ? 100 : pct;
  uint8_t duty = static_cast<uint8_t>((static_cast<uint16_t>(g_fanPct) * 255U) / 100U);
  if (g_fanPwmActiveLow) {
    duty = static_cast<uint8_t>(255U - duty);
  }
  g_fanPwmDutyRaw = duty;
  ledcWrite(FAN_PWM_CHANNEL, g_fanPwmDutyRaw);
  g_fanRpm = static_cast<uint16_t>((static_cast<uint32_t>(g_fanPct) * FAN_EST_MAX_RPM) / 100U);
}

// 主要功能：把当前关键状态写入 NVS。
// 使用方法：force=true 立即保存；否则按节流周期保存。
static void saveState(bool force) {
  static uint32_t lastSaveMs = 0;
  uint32_t now = millis();
  if (!force && (now - lastSaveMs < STATE_SAVE_INTERVAL_MS)) return;
  lastSaveMs = now;

  prefs.putBool("active", g_dryingActive);
  prefs.putUInt("remain", g_remainingSec);
  prefs.putUInt("preset", static_cast<uint32_t>(g_presetIndex));
  prefs.putUInt("fault", g_faultFlags);
  prefs.putFloat("target", g_targetTempC);
  prefs.putFloat("idletmp", g_idleTempC);
  prefs.putUInt("cfgdur", g_configDurationSec);
  prefs.putFloat("kp", g_pidKp);
  prefs.putFloat("ki", g_pidKi);
  prefs.putFloat("kd", g_pidKd);
  prefs.putBool("usermode", g_userCustomMode);
  prefs.putUChar("idlefan", g_idleFanPct);
  prefs.putBool("fanpwmlow", g_fanPwmActiveLow);
  prefs.putUChar("sensortype", static_cast<uint8_t>(g_sensorType));
  prefs.putUChar("cfanbase", g_customFanBasePct);
  prefs.putUChar("cfanmax", g_customFanMaxPct);
  prefs.putFloat("humistop", g_humidityStopPct);
  prefs.putFloat("humidelta", g_humidityStableDeltaPct);
  prefs.putString("actcpid", g_activeCustomPresetId);
  prefs.putString("actcpnm", g_activeCustomPresetName);
}

// 主要功能：从 NVS 读取状态并做断电恢复。
// 使用方法：setup 中调用一次。
static void loadState() {
  g_targetTempC = prefs.getFloat("target", PRESETS[0].targetTempC);
  g_idleTempC = prefs.getFloat("idletmp", IDLE_TEMP_DISABLED_C);
  g_configDurationSec = prefs.getUInt("cfgdur", PRESETS[0].durationSec);
  g_pidKp = prefs.getFloat("kp", PID_DEFAULT_KP);
  g_pidKi = prefs.getFloat("ki", PID_DEFAULT_KI);
  g_pidKd = prefs.getFloat("kd", PID_DEFAULT_KD);
  g_customFanBasePct = prefs.getUChar("cfanbase", PRESETS[0].fanBasePct);
  g_customFanMaxPct = prefs.getUChar("cfanmax", PRESETS[0].fanMaxPct);
  g_humidityStopPct = prefs.getFloat("humistop", 0.0f);
  g_humidityStableDeltaPct = prefs.getFloat("humidelta", HUMIDITY_STABLE_DELTA_PCT);
  g_activeCustomPresetId = prefs.getString("actcpid", "");
  g_activeCustomPresetName = prefs.getString("actcpnm", "");

  bool lastActive = prefs.getBool("active", false);
  uint32_t lastRemain = prefs.getUInt("remain", 0);
  uint32_t lastPreset = prefs.getUInt("preset", 0);
  g_faultFlags = prefs.getUInt("fault", 0);
  g_userCustomMode = prefs.getBool("usermode", false);
  g_idleFanPct = prefs.getUChar("idlefan", 0);
  g_fanPwmActiveLow = prefs.getBool("fanpwmlow", FAN_PWM_ACTIVE_LOW_DEFAULT);
  uint8_t storedSensorType = prefs.getUChar("sensortype", static_cast<uint8_t>(SENSOR_TYPE_AHT10));
  g_sensorType = (storedSensorType == static_cast<uint8_t>(SENSOR_TYPE_SHT3X)) ? SENSOR_TYPE_SHT3X
                                                                                 : SENSOR_TYPE_AHT10;
  g_idleFanPct = constrain(g_idleFanPct, 0, 100);
  g_customFanBasePct = constrain(g_customFanBasePct, 0, 100);
  g_customFanMaxPct = constrain(g_customFanMaxPct, 0, 100);
  if (g_customFanMaxPct < g_customFanBasePct) g_customFanMaxPct = g_customFanBasePct;
  g_humidityStopPct = ClampF(g_humidityStopPct, 0.0f, 100.0f);
  g_humidityStableDeltaPct = ClampF(g_humidityStableDeltaPct, 0.0f, 10.0f);

  if (lastPreset < PRESET_COUNT) g_presetIndex = lastPreset;
  g_targetTempC = ClampF(g_targetTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  if (g_idleTempC < TARGET_TEMP_MIN_C) {
    g_idleTempC = IDLE_TEMP_DISABLED_C;
  } else {
    g_idleTempC = ClampF(g_idleTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  }
  g_configDurationSec = constrain(g_configDurationSec, DURATION_MIN_SEC, DURATION_MAX_SEC);

  if (lastActive && lastRemain > 0) {
    g_dryingActive = true;
    g_remainingSec = lastRemain;
    Serial.println("断电恢复：已继续上次烘干任务。");
  } else {
    g_dryingActive = false;
    g_remainingSec = g_configDurationSec;
  }
}

// 主要功能：创建日志文件并写入表头。
// 使用方法：SPIFFS 初始化成功后调用一次。
static void ensureLogFile() {
  if (!SPIFFS.exists("/dryer_log.csv")) {
    File f = SPIFFS.open("/dryer_log.csv", FILE_WRITE);
    if (f) {
      f.println("ms,temp_c,humidity,target_c,heater_pct,fan_pct,active,fault,preset,remaining_s");
      f.close();
    }
  }
}

// 主要功能：追加运行日志。
// 使用方法：主循环按固定周期调用。
static void appendLogLine() {
  File f = SPIFFS.open("/dryer_log.csv", FILE_APPEND);
  if (!f) return;
  f.printf("%lu,%.2f,%.2f,%.2f,%u,%u,%u,%lu,%s,%lu\n",
           static_cast<unsigned long>(millis()), g_smoothTempC, g_smoothHumi, g_targetTempC,
           RatioToPct(g_heaterDemand), g_fanPct, g_dryingActive ? 1 : 0,
           static_cast<unsigned long>(g_faultFlags), currentProfileName(),
           static_cast<unsigned long>(g_remainingSec));
  f.close();
}

// 主要功能：置位故障并执行安全停机。
// 使用方法：传入故障类型与原因说明。
static void setFault(FaultFlags fault, const char* reason) {
  markStatusDirty();
  g_dryingStartMs = 0;
  sendBemfaNotification("fault", String("故障：") + reason);
  if ((g_faultFlags & fault) == 0) {
    Serial.print("故障：");
    Serial.println(reason);
  }
  g_faultFlags |= fault;
  g_dryingActive = false;
  g_heaterDemand = 0.0f;
  setHeater(false);
}

// 主要功能：清除故障标志。
// 使用方法：确认安全后调用。
static void clearFaults() {
  g_faultFlags = FAULT_NONE;
  markStatusDirty();
}

// 主要功能：返回当前温湿度传感器类型文本。
// 使用方法：Web 状态回传、调试日志与配置保存时调用。
static const char* sensorTypeToString(SensorType type) {
  return (type == SENSOR_TYPE_SHT3X) ? "sht3x" : "aht10";
}

// 主要功能：按当前配置初始化温湿度传感器。
// 使用方法：setup 启动阶段和网页切换传感器类型时调用。
static bool initTempHumiditySensor() {
  bool ok = false;
  const char* reason = nullptr;
  if (g_sensorType == SENSOR_TYPE_SHT3X) {
    ok = sht3x.begin(0x44);
    if (!ok) ok = sht3x.begin(0x45);
    if (ok) {
      sht3x.heater(false);
      Serial.println("SHT3X 初始化成功。");
    } else {
      reason = "SHT3X 初始化失败";
    }
  } else {
    ok = aht10.begin(&Wire);
    if (ok) {
      Serial.println("AHT10 初始化成功。");
    } else {
      reason = "AHT10 初始化失败";
    }
  }

  g_sensorOk = ok;
  if (ok) {
    g_sensorFailCount = 0;
    g_smoothInited = false;
    g_faultFlags &= ~FAULT_SENSOR;
  } else {
    setFault(FAULT_SENSOR, reason ? reason : "传感器初始化失败");
  }
  return ok;
}

// 主要功能：读取一次温湿度采样值。
// 使用方法：主循环采样阶段调用，按传感器类型分发底层读取。
static bool readTempHumidity(float& outTempC, float& outHumiPct) {
  if (g_sensorType == SENSOR_TYPE_SHT3X) {
    float t = sht3x.readTemperature();
    float h = sht3x.readHumidity();
    if (isnan(t) || isnan(h)) return false;
    outTempC = t;
    outHumiPct = h;
    return true;
  }

  sensors_event_t humEvent;
  sensors_event_t tempEvent;
  bool ok = aht10.getEvent(&humEvent, &tempEvent);
  if (!ok) return false;
  outTempC = tempEvent.temperature;
  outHumiPct = humEvent.relative_humidity;
  return true;
}

// 主要功能：应用预设（温度、时长、风扇策略索引）。
// 使用方法：菜单/串口选择预设后调用。
static void applyPreset(size_t presetIndex) {
  if (presetIndex >= PRESET_COUNT) return;
  g_presetIndex = presetIndex;
  g_presetGroupFilter = presetGroupOf(presetIndex);
  g_userCustomMode = false;
  g_targetTempC = PRESETS[presetIndex].targetTempC;
  g_configDurationSec = PRESETS[presetIndex].durationSec;
  g_customFanBasePct = PRESETS[presetIndex].fanBasePct;
  g_customFanMaxPct = PRESETS[presetIndex].fanMaxPct;
  g_humidityStopPct = 0.0f;
  clearActiveCustomPreset();
  markStatusDirty();
}

// 主要功能：启动烘干任务。
// 使用方法：开始前会清故障并重置 PID 内部状态。
static void startDrying() {
  g_dryingStartMs = millis();
  g_lastReachedSetpointMs = 0;
  g_heaterDemandZeroSinceMs = 0;
  markStatusDirty();
  clearFaults();
  g_dryingActive = true;
  g_remainingSec = g_configDurationSec;
  g_pidIntegral = 0.0f;
  g_pidInited = false;
  Serial.println("开始烘干。");
  saveState(true);
}

// 主要功能：停止烘干任务。
// 使用方法：手动停止、倒计时结束或故障触发时调用。
static void stopDrying() {
  g_dryingStartMs = 0;
  g_lastReachedSetpointMs = 0;
  g_heaterDemandZeroSinceMs = 0;
  markStatusDirty();
  g_dryingActive = false;
  g_heaterDemand = 0.0f;
  setHeater(false);
  Serial.println("已停止烘干。");
  saveState(true);
}

// 主要功能：启动 PID 自动校准（继电振荡法）。
// 使用方法：菜单或串口触发；校准期间会临时接管加热输出。
static void startPidAutoTune() {
  g_dryingStartMs = millis();
  g_lastReachedSetpointMs = 0;
  g_heaterDemandZeroSinceMs = 0;
  if (!g_sensorOk) {
    Serial.println("PID自校准失败：传感器不可用。");
    return;
  }
  clearFaults();
  g_pidAutoTuneActive = true;
  StartPidAutoTune(g_pidAutoTuneState, g_targetTempC, g_smoothTempC, millis());
  snprintf(g_pidAutoTuneMsg, sizeof(g_pidAutoTuneMsg), "RUN");
  g_pidInited = false;
  g_pidIntegral = 0.0f;
  g_dryingActive = true;
  Serial.println("PID自校准已启动（继电振荡法）。");
}

// 主要功能：初始化按钮输入状态。
// 使用方法：setup 中调用一次。
static void initButtons() {
  for (uint8_t i = 0; i < BTN_COUNT; ++i) {
    pinMode(g_buttons[i].pin, INPUT_PULLUP);
    bool lv = digitalRead(g_buttons[i].pin);
    g_buttons[i].stableLevel = lv;
    g_buttons[i].lastRead = lv;
    g_buttons[i].lastChangeMs = millis();
  }
}

// 主要功能：读取去抖后的按键“按下事件”（边沿触发）。
// 使用方法：loop 中频繁调用；返回 true 表示本次新按下。
static bool pollButtonPressed(ButtonId id) {
  ButtonState& b = g_buttons[id];
  bool rd = digitalRead(b.pin);
  uint32_t now = millis();

  if (rd != b.lastRead) {
    b.lastRead = rd;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) >= BUTTON_DEBOUNCE_MS && b.stableLevel != rd) {
    b.stableLevel = rd;
    if (b.stableLevel == LOW) {
      return true;
    }
  }
  return false;
}

// 主要功能：执行一条控制台命令并返回文本回显。
// 使用方法：串口与 Web 调试接口共用，传入完整命令行字符串。
static String executeConsoleCommand(const String& cmdLine) {
  String cmd = cmdLine;
  cmd.trim();
  if (cmd.length() == 0) return "";

  String lower = cmd;
  lower.toLowerCase();

  if (lower == "help") {
    return "命令：help | start | stop | preset <name> | idletemp <0|35~120> | sensor <aht10|sht3x> | fanpwm <low|high> | status | faultreset | autotune | wifistatus | wifiap | wificlear | btn <up|down|left|right|ok>\n";
  }
  if (lower == "start") {
    startDrying();
    return "开始烘干。\n";
  }
  if (lower == "stop") {
    stopDrying();
    return "已停止烘干。\n";
  }
  if (lower.startsWith("preset ")) {
    String name = cmd.substring(7);
    int idx = findPresetByName(name);
    if (idx >= 0) {
      applyPreset(static_cast<size_t>(idx));
      saveState(true);
      return String("已设置预设：") + activePreset().name + "\n";
    }
    return "未知材料预设。\n";
  }
  if (lower.startsWith("idletemp ")) {
    String value = lower.substring(9);
    value.trim();
    float t = value.toFloat();
    if (t < TARGET_TEMP_MIN_C) {
      g_idleTempC = IDLE_TEMP_DISABLED_C;
      saveState(true);
      return "闲时温度已关闭。\n";
    }
    g_idleTempC = ClampF(t, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
    saveState(true);
    char line[64];
    snprintf(line, sizeof(line), "闲时温度已设置为 %.1fC\n", g_idleTempC);
    return String(line);
  }
  if (lower.startsWith("sensor ")) {
    String sensor = lower.substring(7);
    sensor.trim();
    if (sensor == "aht10") {
      g_sensorType = SENSOR_TYPE_AHT10;
    } else if (sensor == "sht3x" || sensor == "sht31" || sensor == "sht30") {
      g_sensorType = SENSOR_TYPE_SHT3X;
    } else {
      return "传感器类型无效，仅支持 aht10/sht3x。\n";
    }
    bool ok = initTempHumiditySensor();
    saveState(true);
    return ok ? String("传感器已切换：") + sensorTypeToString(g_sensorType) + "\n"
              : String("传感器切换失败：") + sensorTypeToString(g_sensorType) + "\n";
  }
  if (lower.startsWith("fanpwm ")) {
    String mode = lower.substring(7);
    mode.trim();
    if (mode == "low" || mode == "invert" || mode == "active_low") {
      g_fanPwmActiveLow = true;
    } else if (mode == "high" || mode == "normal" || mode == "active_high") {
      g_fanPwmActiveLow = false;
    } else {
      return "fanpwm 参数无效，仅支持 low/high。\n";
    }
    setFanPct(g_fanPct);
    saveState(true);
    return String("风扇PWM极性已切换：") + (g_fanPwmActiveLow ? "有效低电平" : "有效高电平") + "\n";
  }
  if (lower == "faultreset") {
    clearFaults();
    saveState(true);
    return "故障已清除。\n";
  }
  if (lower == "status") {
    char line[288];
    snprintf(line, sizeof(line),
             "预设:%s 运行:%u 温度:%.2fC 湿度:%.2f%% 目标:%.1fC 闲时:%.1fC 加热:%u%% 风扇:%u%%/%uRPM PWM:%s 传感器:%s 故障:%lu 剩余:%lu\n",
             currentProfileName(), g_dryingActive ? 1 : 0, g_smoothTempC, g_smoothHumi, g_targetTempC,
             g_idleTempC, RatioToPct(g_heaterDemand), g_fanPct, g_fanRpm,
             g_fanPwmActiveLow ? "active_low" : "active_high", sensorTypeToString(g_sensorType),
             static_cast<unsigned long>(g_faultFlags), static_cast<unsigned long>(g_remainingSec));
    return String(line);
  }
  if (lower == "autotune") {
    if (!g_sensorOk) {
      return "PID自校准失败：传感器不可用。\n";
    }
    startPidAutoTune();
    return "PID自校准已启动（继电振荡法）。\n";
  }
  if (lower == "wifistatus") {
    char line[160];
    snprintf(line, sizeof(line), "WiFi:%s AP:%u IP:%s SSID:%s\n", g_wifiConnected ? "已连接" : "未连接",
             g_apConfigMode ? 1 : 0,
             g_wifiConnected ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
             g_wifiConnected ? WiFi.SSID().c_str() : "(none)");
    return String(line);
  }
  if (lower == "wifiap") {
    startConfigApPortal();
    return "已切换到 AP 配网模式。\n";
  }
  if (lower == "wificlear") {
    clearWifiConfig();
    WiFi.disconnect(true, true);
    startConfigApPortal();
    return "WiFi 凭据已清除，已进入 AP 配网模式。\n";
  }
  if (lower.startsWith("btn ")) {
    String key = lower.substring(4);
    key.trim();
    if (key == "up") {
      handleButtonEvents(true, false, false, false, false);
      return "虚拟按键：UP\n";
    }
    if (key == "down") {
      handleButtonEvents(false, true, false, false, false);
      return "虚拟按键：DOWN\n";
    }
    if (key == "left") {
      handleButtonEvents(false, false, true, false, false);
      return "虚拟按键：LEFT\n";
    }
    if (key == "right") {
      handleButtonEvents(false, false, false, true, false);
      return "虚拟按键：RIGHT\n";
    }
    if (key == "ok" || key == "center" || key == "middle") {
      handleButtonEvents(false, false, false, false, true);
      return "虚拟按键：OK\n";
    }
    return "按键参数错误，用法：btn <up|down|left|right|ok>\n";
  }
  if (lower == "up" || lower == "down" || lower == "left" || lower == "right" || lower == "ok") {
    return executeConsoleCommand(String("btn ") + lower);
  }
  return "未知命令，请输入 help 查看帮助。\n";
}

// 主要功能：处理串口命令（保留调试与远程控制）。
// 使用方法：loop 中周期调用。
static void processSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  String out = executeConsoleCommand(cmd);
  if (out.length() > 0) Serial.print(out);
}

// 主要功能：处理一次按键事件（来自实体按键或虚拟按键）。
// 使用方法：将本次触发的按键置为 true，其余置 false。
static void handleButtonEvents(bool up, bool down, bool left, bool right, bool ok) {
  if (!(up || down || left || right || ok)) return;

  if (g_uiMode == UI_HOME) {
    if (ok) {
      g_uiMode = UI_MENU;
      g_menuIndex = 0;
      g_mainMenuFx.startEnter();
      g_mainMenuFx.setSelection(g_menuIndex, true);
    }
    if (left || right) {
      g_showTempPage = !g_showTempPage;
    }
    if (up) {
      g_targetTempC = ClampF(g_targetTempC + 0.5f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      g_userCustomMode = true;
      saveState(true);
    }
    if (down) {
      g_targetTempC = ClampF(g_targetTempC - 0.5f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      g_userCustomMode = true;
      saveState(true);
    }
    return;
  }

  if (g_uiMode == UI_MENU) {
    if (g_mainMenuFx.isExiting()) return;

    if (left) {
      g_menuIndex = (g_menuIndex == 0) ? (MAIN_MENU_COUNT - 1) : (g_menuIndex - 1);
      g_mainMenuFx.nudge(-1);
      g_mainMenuFx.setSelection(g_menuIndex, false);
    }
    if (right) {
      g_menuIndex = (g_menuIndex + 1) % MAIN_MENU_COUNT;
      g_mainMenuFx.nudge(+1);
      g_mainMenuFx.setSelection(g_menuIndex, false);
    }
    if (down) {
      g_mainMenuFx.startExit();
      return;
    }
    if (up) {
      if (g_dryingActive) {
        stopDrying();
      } else {
        startDrying();
      }
      g_mainMenuFx.startExit();
      return;
    }
    if (ok) {
      switch (g_menuIndex) {
        case 0:
          if (g_dryingActive) {
            stopDrying();
          } else {
            startDrying();
          }
          g_mainMenuFx.startExit();
          break;
        case 1:
          g_uiMode = UI_SET_TEMP;
          break;
        case 2:
          g_uiMode = UI_SET_TIME;
          break;
        case 3:
          g_uiMode = UI_SET_IDLE_TEMP;
          break;
        case 4:
          g_uiMode = UI_SET_IDLE_FAN;
          break;
        case 5:
          g_userCustomMode = true;
          g_uiMode = UI_SET_TEMP;
          break;
        case 6:
          g_uiMode = UI_SET_PRESET;
          break;
        case 7:
          g_uiMode = UI_SET_PID;
          g_pidEditIndex = 0;
          break;
        case 8:
          startPidAutoTune();
          g_uiMode = UI_HOME;
          break;
        case 9:
          clearFaults();
          saveState(true);
          g_uiMode = UI_HOME;
          break;
      }
    }
    return;
  }

  if (g_uiMode == UI_SET_TEMP) {
    float prevValue = g_targetTempC;
    bool changed = false;
    if (up) {
      g_targetTempC = ClampF(g_targetTempC + 0.5f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (down) {
      g_targetTempC = ClampF(g_targetTempC - 0.5f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (right) {
      g_targetTempC = ClampF(g_targetTempC + 2.0f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (left) {
      g_targetTempC = ClampF(g_targetTempC - 2.0f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (changed) {
      g_userCustomMode = true;
      startNumberRailAnim(g_animSetTempRail, prevValue, g_targetTempC);
    }
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_TIME) {
    uint32_t prevValue = g_configDurationSec;
    bool changed = false;
    if (up) {
      g_configDurationSec = SaturatingAddU32(g_configDurationSec, 10U * 60U, DURATION_MAX_SEC);
      changed = true;
    }
    if (down) {
      g_configDurationSec = SaturatingSubU32(g_configDurationSec, 10U * 60U, DURATION_MIN_SEC);
      changed = true;
    }
    if (right) {
      g_configDurationSec = SaturatingAddU32(g_configDurationSec, 60U * 60U, DURATION_MAX_SEC);
      changed = true;
    }
    if (left) {
      g_configDurationSec = SaturatingSubU32(g_configDurationSec, 60U * 60U, DURATION_MIN_SEC);
      changed = true;
    }
    if (changed) {
      g_userCustomMode = true;
      char oldText[24];
      char newText[24];
      formatSetTimeText(prevValue, oldText, sizeof(oldText));
      formatSetTimeText(g_configDurationSec, newText, sizeof(newText));
      int8_t dir = (g_configDurationSec >= prevValue) ? 1 : -1;
      startValueSlide(g_animSetTime, oldText, newText, dir);
    }
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_IDLE_TEMP) {
    float prevValue = g_idleTempC;
    bool changed = false;
    if (up) {
      if (g_idleTempC < TARGET_TEMP_MIN_C) g_idleTempC = TARGET_TEMP_MIN_C;
      g_idleTempC = ClampF(g_idleTempC + 0.5f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (down) {
      if (g_idleTempC <= TARGET_TEMP_MIN_C) {
        g_idleTempC = IDLE_TEMP_DISABLED_C;
      } else {
        g_idleTempC = g_idleTempC - 0.5f;
        if (g_idleTempC < TARGET_TEMP_MIN_C) g_idleTempC = IDLE_TEMP_DISABLED_C;
      }
      changed = true;
    }
    if (right) {
      if (g_idleTempC < TARGET_TEMP_MIN_C) g_idleTempC = TARGET_TEMP_MIN_C;
      g_idleTempC = ClampF(g_idleTempC + 2.0f, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
      changed = true;
    }
    if (left) {
      if (g_idleTempC <= TARGET_TEMP_MIN_C) {
        g_idleTempC = IDLE_TEMP_DISABLED_C;
      } else {
        g_idleTempC = g_idleTempC - 2.0f;
        if (g_idleTempC < TARGET_TEMP_MIN_C) g_idleTempC = IDLE_TEMP_DISABLED_C;
      }
      changed = true;
    }
    if (changed) {
      if (g_idleTempC < TARGET_TEMP_MIN_C) {
        g_animIdleTempRail.active = false;
      } else {
        float oldForAnim = (prevValue < TARGET_TEMP_MIN_C) ? TARGET_TEMP_MIN_C : prevValue;
        startNumberRailAnim(g_animIdleTempRail, oldForAnim, g_idleTempC);
      }
    }
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_IDLE_FAN) {
    uint8_t prevValue = g_idleFanPct;
    if (up) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) + 1, 0, 100);
    if (down) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) - 1, 0, 100);
    if (right) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) + 10, 0, 100);
    if (left) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) - 10, 0, 100);
    if (g_idleFanPct != prevValue) {
      char oldText[24];
      char newText[24];
      formatIdleFanText(prevValue, oldText, sizeof(oldText));
      formatIdleFanText(g_idleFanPct, newText, sizeof(newText));
      int8_t dir = (g_idleFanPct >= prevValue) ? 1 : -1;
      startValueSlide(g_animIdleFan, oldText, newText, dir);
    }
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_PRESET) {
    if (up) {
      uint8_t g = (g_presetGroupFilter == 0) ? (GROUP_COUNT - 1) : (g_presetGroupFilter - 1);
      switchPresetGroup(g, -1);
    }
    if (down) {
      uint8_t g = (g_presetGroupFilter + 1) % GROUP_COUNT;
      switchPresetGroup(g, +1);
    }
    if (left) {
      g_presetIndex = nextPresetInGroup(g_presetIndex, -1, g_presetGroupFilter);
      applyPreset(g_presetIndex);
    }
    if (right) {
      g_presetIndex = nextPresetInGroup(g_presetIndex, +1, g_presetGroupFilter);
      applyPreset(g_presetIndex);
    }
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_PID) {
    float* p = nullptr;
    float step = 0.01f;
    if (g_pidEditIndex == 0) {
      p = &g_pidKp;
      step = 0.01f;
    } else if (g_pidEditIndex == 1) {
      p = &g_pidKi;
      step = 0.001f;
    } else {
      p = &g_pidKd;
      step = 0.01f;
    }

    if (up) *p = *p + step;
    if (down) *p = (*p > step) ? (*p - step) : 0.0f;
    if (left) {
      if (g_pidEditIndex == 0)
        g_pidEditIndex = 2;
      else
        g_pidEditIndex--;
    }
    if (right) g_pidEditIndex = (g_pidEditIndex + 1) % 3;
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
  }
}

// 主要功能：五键菜单交互逻辑。
// 使用方法：loop 中调用，处理菜单切换、温度/PID 参数调整、启停等。
static void processButtonUi() {
  bool up = pollButtonPressed(BTN_UP);
  bool down = pollButtonPressed(BTN_DOWN);
  bool left = pollButtonPressed(BTN_LEFT);
  bool right = pollButtonPressed(BTN_RIGHT);
  bool ok = pollButtonPressed(BTN_OK);
  handleButtonEvents(up, down, left, right, ok);
}

// 主要功能：PID 自动调温计算。
// 使用方法：在控制周期调用，输出 0~1 加热需求。
static void updatePidControl() {
  if (g_pidAutoTuneActive) {
    PidAutoTuneStepResult r = StepPidAutoTune(
        g_pidAutoTuneState, AUTOTUNE_CFG, millis(), g_smoothTempC, g_sensorOk,
        (g_faultFlags != FAULT_NONE), g_heaterDemand, g_pidKp, g_pidKi, g_pidKd,
        0.01f, 3.00f, 0.0001f, 0.20f, 0.00f, 20.0f);
    if (r == PidAutoTuneStepResult::Done) {
      g_pidAutoTuneActive = false;
      g_pidInited = false;
      snprintf(g_pidAutoTuneMsg, sizeof(g_pidAutoTuneMsg), "DONE");
      Serial.printf("PID自校准完成：Kp=%.4f Ki=%.4f Kd=%.4f\n", g_pidKp, g_pidKi, g_pidKd);
      saveState(true);
    } else if (r == PidAutoTuneStepResult::Fail) {
      g_pidAutoTuneActive = false;
      g_heaterDemand = 0.0f;
      snprintf(g_pidAutoTuneMsg, sizeof(g_pidAutoTuneMsg), "FAIL");
      Serial.println("PID自校准失败：振荡条件不满足或超时。");
    }
    return;
  }

  bool idleHeat = idleHeatEnabled();
  if ((!g_dryingActive && !idleHeat) || g_faultFlags != FAULT_NONE || !g_sensorOk) {
    g_heaterDemand = 0.0f;
    return;
  }

  constexpr float DT = CONTROL_INTERVAL_MS / 1000.0f;
  float setpoint = g_dryingActive ? g_targetTempC : g_idleTempC;
  float error = setpoint - g_smoothTempC;

  if (!g_pidInited) {
    g_pidPrevTemp = g_smoothTempC;
    g_pidIntegral = 0.0f;
    g_pidInited = true;
  }

  g_pidIntegral = ClampF(g_pidIntegral + error * DT, -80.0f, 80.0f);
  float dTemp = (g_smoothTempC - g_pidPrevTemp) / DT;
  g_pidPrevTemp = g_smoothTempC;

  float out = g_pidKp * error + g_pidKi * g_pidIntegral - g_pidKd * dTemp;
  g_heaterDemand = ClampF(out, 0.0f, 1.0f);
}

// 主要功能：风扇策略控制。
// 使用方法：控制周期调用，自动按工况调整风扇占空比。
static void updateFanStrategy() {
  if (g_pidAutoTuneActive) {
    // 自动校准期间固定风扇，减少扰动对振荡测量的影响。
    setFanPct(currentFanBasePct());
    return;
  }

  if (g_faultFlags & FAULT_OVER_TEMP) {
    setFanPct(100);
    return;
  }

  if (!g_dryingActive) {
    setFanPct(g_idleFanPct);
    return;
  }

  uint8_t heaterPct = RatioToPct(g_heaterDemand);
  uint8_t basePct = currentFanBasePct();
  uint8_t maxPct = currentFanMaxPct();
  int boosted = basePct + static_cast<int>(heaterPct / 2);
  if (g_smoothTempC > g_targetTempC) boosted += 10;
  if (boosted > maxPct) boosted = maxPct;
  if (boosted < basePct) boosted = basePct;
  setFanPct(static_cast<uint8_t>(boosted));
}

// 主要功能：把加热需求转换为时间比例开关。
// 使用方法：loop 中持续调用。
static void updateHeaterWindow() {
  static uint32_t windowStartMs = 0;
  uint32_t now = millis();
  if (now - windowStartMs >= CONTROL_WINDOW_MS) {
    windowStartMs = now;
  }
  bool idleHeat = idleHeatEnabled();
  if ((!g_dryingActive && !idleHeat) || g_faultFlags != FAULT_NONE || !g_sensorOk) {
    setHeater(false);
    return;
  }

  uint32_t onMs = static_cast<uint32_t>(g_heaterDemand * CONTROL_WINDOW_MS);
  setHeater((now - windowStartMs) < onMs);
}

// 主要功能：根据 UI 模式绘制 OLED。
// 使用方法：当状态变化或到达刷新周期时调用。
static void drawHomeLayer(int16_t yOffset) {
  oled.setFont(u8g2_font_wqy12_t_gb2312);
  oled.drawUTF8(0, 11 + yOffset, g_showTempPage ? "温度" : "湿度");
  String ipTop = g_wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  oled.setFont(u8g2_font_4x6_tf);
  oled.drawStr(68, 8 + yOffset, ipTop.c_str());
  oled.drawHLine(0, 14 + yOffset, 128);

  oled.setFont(u8g2_font_logisoso18_tf);
  char mainText[24];
  if (!g_sensorOk) {
    snprintf(mainText, sizeof(mainText), "Err");
  } else if (g_showTempPage) {
    snprintf(mainText, sizeof(mainText), "%.1f C", g_smoothTempC);
  } else {
    snprintf(mainText, sizeof(mainText), "%.1f %%", g_smoothHumi);
  }
  oled.drawStr(0, 38 + yOffset, mainText);

  char line2[48];
  char remain[12];
  FormatDuration(g_remainingSec, remain, sizeof(remain));
  snprintf(line2, sizeof(line2), "P:%s T:%.1fC %s", currentProfileName(), g_targetTempC, remain);
  if (g_userCustomMode) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 50 + yOffset, line2);
    oled.setFont(u8g2_font_5x8_tf);
  } else {
    oled.setFont(u8g2_font_5x8_tf);
    oled.drawStr(0, 50 + yOffset, line2);
  }

  char line3[48];
  if (g_faultFlags != FAULT_NONE) {
    snprintf(line3, sizeof(line3), "FAULT:%s", faultFlagsToShortText(g_faultFlags));
  } else if (g_pidAutoTuneActive) {
    snprintf(line3, sizeof(line3), "PID-TUNE %s", g_pidAutoTuneMsg);
  } else if (!g_wifiConnected) {
    if (g_apConfigMode) {
      snprintf(line3, sizeof(line3), "WIFI:AP %s", WiFi.softAPIP().toString().c_str());
    } else {
      snprintf(line3, sizeof(line3), "WIFI:DISCONNECTED");
    }
  } else {
    const char* mode = g_dryingActive ? "RUN" : (idleHeatEnabled() ? "IDLEHEAT" : "STOP");
    snprintf(line3, sizeof(line3), "H:%u F:%u %s", RatioToPct(g_heaterDemand), g_fanPct, mode);
  }
  oled.drawStr(0, 61 + yOffset, line3);
}

static void drawScreen() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);

  if (g_uiMode == UI_HOME) {
    drawHomeLayer(0);
  } else if (g_uiMode == UI_MENU) {
    int16_t menuY = g_mainMenuFx.menuOffsetY();
    drawHomeLayer(menuY - 64);
    g_mainMenuFx.render(oled);
  } else if (g_uiMode == UI_SET_TEMP) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "设定目标温度");
    drawTemperatureRail(g_animSetTempRail, g_targetTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  } else if (g_uiMode == UI_SET_TIME) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "设定烘干时长");
    char d[16];
    formatSetTimeText(g_configDurationSec, d, sizeof(d));
    oled.setFont(u8g2_font_logisoso18_tf);
    drawValueWithSlide(g_animSetTime, d, 0, 52, 12);
  } else if (g_uiMode == UI_SET_IDLE_TEMP) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "设定闲时温度");
    if (g_idleTempC < TARGET_TEMP_MIN_C) {
      const int16_t baselineY = 52;
      drawFocusCard(baselineY, 82, 32, 6);
      oled.setDrawColor(0);
      oled.setFont(u8g2_font_logisoso24_tf);
      drawCenteredText("OFF", baselineY);
      oled.setDrawColor(1);
    } else {
      drawTemperatureRail(g_animIdleTempRail, g_idleTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
    }
  } else if (g_uiMode == UI_SET_IDLE_FAN) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "设定空闲风扇");
    char f[24];
    formatIdleFanText(g_idleFanPct, f, sizeof(f));
    oled.setFont(u8g2_font_logisoso18_tf);
    drawValueWithSlide(g_animIdleFan, f, 0, 52, 12);
  } else if (g_uiMode == UI_SET_PRESET) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "选择材料预设");
    char gline[24];
    snprintf(gline, sizeof(gline), "组:%s", PRESET_GROUP_NAMES[g_presetGroupFilter]);
    int16_t glineW = oled.getUTF8Width(gline);
    oled.drawUTF8((128 - glineW) / 2, 24, gline);
    drawFocusCard(50, 102, 32, 6);
    oled.setDrawColor(0);
    oled.setFont(u8g2_font_logisoso18_tf);
    drawCenteredText(activePreset().name, 50);
    oled.setDrawColor(1);
    char p[20];
    snprintf(p, sizeof(p), "T%.1fC", g_targetTempC);
    oled.setFont(u8g2_font_6x12_tf);
    drawCenteredText(p, 62);
  } else if (g_uiMode == UI_SET_PID) {
    const char* pidName[3] = {"Kp", "Ki", "Kd"};
    float pidVal[3] = {g_pidKp, g_pidKi, g_pidKd};
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 12, "PID参数");
    char tag[16];
    snprintf(tag, sizeof(tag), "当前:%s", pidName[g_pidEditIndex]);
    int16_t tagW = oled.getUTF8Width(tag);
    oled.drawUTF8((128 - tagW) / 2, 24, tag);
    drawFocusCard(50, 102, 32, 6);
    oled.setDrawColor(0);
    oled.setFont(u8g2_font_logisoso18_tf);
    char v[24];
    snprintf(v, sizeof(v), "%s=%.3f", pidName[g_pidEditIndex], pidVal[g_pidEditIndex]);
    drawCenteredText(v, 50);
    oled.setDrawColor(1);
    oled.setFont(u8g2_font_5x8_tf);
    char nav[28];
    snprintf(nav, sizeof(nav), "%s %s %s", g_pidEditIndex == 0 ? "[Kp]" : " Kp ",
             g_pidEditIndex == 1 ? "[Ki]" : " Ki ", g_pidEditIndex == 2 ? "[Kd]" : " Kd ");
    drawCenteredText(nav, 62);
  }

  oled.sendBuffer();
}

// 主要功能：系统初始化。
// 使用方法：Arduino 自动调用一次。
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("烘干箱系统启动中...");

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ_HZ, FAN_PWM_RESOLUTION_BITS);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  setFanPct(20);

  initButtons();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  oled.setI2CAddress(OLED_ADDR_8BIT);
  oled.begin();
  oled.enableUTF8Print();
  oled.setContrast(255);

  bool fsOk = SPIFFS.begin(true);
  Serial.println(fsOk ? "SPIFFS 初始化成功。" : "SPIFFS 初始化失败。");
  if (fsOk) {
    ensureLogFile();
    loadUserPresets();
  }

  prefs.begin("dryer", false);
  loadState();
  g_mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  g_webToken = prefs.getString("webtoken", "");
  g_bemfaUid = prefs.getString("bemfa_uid", "");
  g_bemfaMqttKey = prefs.getString("bemfa_mkey", "");
  g_bemfaControlTopic = prefs.getString("bemfa_ctl", g_bemfaControlTopic);
  g_bemfaStatusTopic = prefs.getString("bemfa_sta", g_bemfaStatusTopic);
  g_bemfaOtaTopic = prefs.getString("bemfa_ota", "");
  g_bemfaNotificationsEnabled = prefs.getBool("notify_en", false);
  g_bemfaNotifyGroup = prefs.getString("notify_grp", "default");
  g_otaAutoCheckEnabled = prefs.getBool("ota_auto", false);
  Serial.printf("[MQTT] buffer size=%u\n", static_cast<unsigned>(g_mqttClient.getBufferSize()));
  printBemfaMqttConfig("config loaded");
  g_animSetTempRail.oldValue = g_targetTempC;
  g_animSetTempRail.newValue = g_targetTempC;
  g_animSetTempRail.progress = 1.0f;
  g_animSetTempRail.active = false;
  float idleInitTemp = (g_idleTempC < TARGET_TEMP_MIN_C) ? TARGET_TEMP_MIN_C : g_idleTempC;
  g_animIdleTempRail.oldValue = idleInitTemp;
  g_animIdleTempRail.newValue = idleInitTemp;
  g_animIdleTempRail.progress = 1.0f;
  g_animIdleTempRail.active = false;
  loadWifiConfig();
  setFanPct(g_idleFanPct);
  g_mainMenuFx.begin(MAIN_MENU_ITEMS, MAIN_MENU_COUNT);
  g_mainMenuFx.setSelection(g_menuIndex, true);

  initTempHumiditySensor();

  if (!connectWifiSta()) {
    startConfigApPortal();
  }
  g_lastOtaCheckMs = millis();
  startWebServer();

  drawScreen();
  saveState(true);
}

// 主要功能：主循环调度。
// 使用方法：Arduino 自动反复调用。
void loop() {
  static uint32_t lastSensorMs = 0;
  static uint32_t lastControlMs = 0;
  static uint32_t lastDisplayMs = 0;
  static uint32_t lastPageMs = 0;
  static uint32_t lastTickMs = 0;
  static uint32_t lastLogMs = 0;

  uint32_t now = millis();
  bool needRedraw = false;

  // A. 输入层：串口命令 + 五键菜单。
  serviceNetwork();
  processSerialCommands();
  processButtonUi();
  updateSettingAnimations(now);
  if (g_uiMode == UI_MENU) {
    g_mainMenuFx.setSelection(g_menuIndex, false);
    if (g_mainMenuFx.tick(now)) needRedraw = true;
    if (g_mainMenuFx.isExitFinished()) {
      g_uiMode = UI_HOME;
      needRedraw = true;
    }
  }

  // B. 采样层：读取温湿度，做滤波与故障检测。
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    float sampleTempC = 0.0f;
    float sampleHumiPct = 0.0f;
    bool ok = readTempHumidity(sampleTempC, sampleHumiPct);
    if (!ok) {
      if (g_sensorFailCount < 255) g_sensorFailCount++;
      if (g_sensorFailCount >= MAX_SENSOR_FAIL_COUNT) {
        g_sensorOk = false;
        setFault(FAULT_SENSOR, "传感器读取超时");
      }
    } else {
      g_sensorOk = true;
      g_sensorFailCount = 0;
      g_tempC = sampleTempC;
      g_humi = sampleHumiPct;
      if (!g_smoothInited) {
        g_smoothTempC = g_tempC;
        g_smoothHumi = g_humi;
        g_smoothInited = true;
      } else {
        g_smoothTempC += SMOOTH_ALPHA * (g_tempC - g_smoothTempC);
        g_smoothHumi += SMOOTH_ALPHA * (g_humi - g_smoothHumi);
      }
      if (g_smoothTempC >= ABS_OVER_TEMP_C) {
        setFault(FAULT_OVER_TEMP, "温度过高");
      }
    }
    needRedraw = true;
  }

  // C. 控制层：PID 自动调温 + 风扇策略 + 状态保存。
  trackHumidityHistory(now);
  updateDryingCompletionByHumidity();
  updateAdvancedFaults(now);
  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updatePidControl();
    updateFanStrategy();
    saveState(false);
    needRedraw = true;
  }

  // D. 执行层：时间比例加热输出。
  updateHeaterWindow();

  // E. 计时层：运行倒计时推进。
  if (now - lastTickMs >= 1000) {
    lastTickMs = now;
    if (!g_pidAutoTuneActive && g_dryingActive && g_remainingSec > 0) {
      g_remainingSec--;
      if (g_remainingSec == 0) {
        stopDrying();
        sendBemfaNotification("warn", "烘干完成：倒计时结束");
      }
    }
    needRedraw = true;
  }

  // F. 日志层：周期写 SPIFFS + 串口状态。
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    appendLogLine();
    Serial.printf("温度:%.2fC 湿度:%.2f%% 目标:%.1fC 预设:%s 剩余:%lus 加热:%u%% 风扇:%u%%/%uRPM 故障:%lu\n",
                  g_smoothTempC, g_smoothHumi, g_targetTempC, currentProfileName(),
                  static_cast<unsigned long>(g_remainingSec), RatioToPct(g_heaterDemand), g_fanPct, g_fanRpm,
                  static_cast<unsigned long>(g_faultFlags));
  }

  // G. 显示层：主页自动轮播温湿度（菜单界面不轮播）。
  if (g_uiMode == UI_HOME && (now - lastPageMs >= PAGE_INTERVAL_MS)) {
    lastPageMs = now;
    g_showTempPage = !g_showTempPage;
    needRedraw = true;
  }

  // H. 刷新层：按节奏重绘 OLED。
  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    needRedraw = true;
  }

  if (needRedraw) {
    drawScreen();
  }

  delay(10);
}
