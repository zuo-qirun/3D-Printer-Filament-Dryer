#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include "dryer_common.h"
#include "pid_autotune.h"

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
  Fan TACH -> 可选测速输入（当前代码未启用）

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

constexpr uint32_t SENSOR_INTERVAL_MS = 500;       // 传感器采样周期（ms）。
constexpr uint32_t CONTROL_INTERVAL_MS = 1000;     // PID/风扇控制周期（ms）。
constexpr uint32_t DISPLAY_INTERVAL_MS = 200;      // OLED 刷新节拍（ms）。
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
};

enum UiMode : uint8_t {
  UI_HOME = 0,
  UI_MENU,
  UI_SET_TEMP,
  UI_SET_TIME,
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

Adafruit_AHTX0 aht10;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;
WebServer g_webServer(80);
DNSServer g_dnsServer;

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

float g_targetTempC = 50.0f;             // 用户设定目标温度（摄氏度）。
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

// 主要功能：获取当前预设。
// 使用方法：读取当前材料的风扇策略参数。
static const MaterialPreset& activePreset() { return PRESETS[g_presetIndex]; }

// 主要功能：获取当前首页/状态显示使用的材料标签。
// 使用方法：当参数处于自定义模式时返回 user(自定义)，否则返回预设名。
static const char* currentProfileName() {
  return g_userCustomMode ? "user" : activePreset().name;
}

// 前置声明：用于分组辅助函数中调用。
static void applyPreset(size_t presetIndex);
static void startDrying();
static void stopDrying();
static void startPidAutoTune();
static void clearFaults();
static void saveState(bool force);
static const char* currentProfileName();

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
static void handleNotFound();
static void serviceNetwork();

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

// 主要功能：在 OLED 上显示当前 Wi-Fi 阶段状态，避免联网阶段黑屏。
// 使用方法：连接路由器/启动 AP 配网时调用。
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

// 主要功能：返回 Web 首页（状态 + 控制 + 配网）。
// 使用方法：浏览器访问设备 IP 根路径触发。
static void handleWebRoot() {
  static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>3D耗材烘干箱</title>
<style>
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
label{display:block;margin:8px 0 4px;font-size:13px}input,select,button{width:100%;padding:9px;border:1px solid #cbd5e1;border-radius:8px;background:#fff}
canvas{width:100%;height:170px;background:#fff;border:1px solid var(--line);border-radius:10px}
.ok{color:var(--good)}.bad{color:var(--bad)}
@media (max-width:1100px){.span6,.span4,.span3{grid-column:span 12}.kpi{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body><div class="wrap">
<div class="head"><h1>3D打印耗材烘干箱</h1><div id="net" class="sub">网络状态加载中...</div></div>
<div class="dash">
  <div class="card span12">
    <div class="kpi">
      <div class="kv"><div class="k">温度</div><div class="v"><span id="temp">--</span> C</div></div>
      <div class="kv"><div class="k">湿度</div><div class="v"><span id="humi">--</span> %</div></div>
      <div class="kv"><div class="k">目标/剩余</div><div class="v"><span id="target">--</span>C / <span id="remain">--</span></div></div>
      <div class="kv"><div class="k">运行/材料</div><div class="v"><span id="active">--</span> / <span id="preset">--</span></div></div>
    </div>
    <div class="sub" style="margin-top:8px">加热 <span id="heater">--</span>% | 风扇转速(占空比) <span id="fan">--</span>% | <span id="fault" class="bad">无故障</span></div>
  </div>
  <div class="card span6 panel"><h2>温度曲线</h2><canvas id="cTemp" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>湿度曲线</h2><canvas id="cHumi" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>风扇转速曲线(%)</h2><canvas id="cFan" width="600" height="170"></canvas></div>
  <div class="card span6 panel"><h2>加热输出曲线(%)</h2><canvas id="cHeat" width="600" height="170"></canvas></div>
  <div class="card span4 panel"><h2>控制</h2>
    <label>参数模式</label><select id="modeSel" onchange="onModeChange()"><option value="custom">user</option><option value="preset">材料预设</option></select>
    <label>目标温度 (35~120 C)</label><input id="setTemp" type="number" step="0.5" min="35" max="120">
    <label>烘干时长 (分钟)</label><input id="setDur" type="number" min="30" max="1440">
    <label>空闲风扇转速 (0~100 %)</label><input id="setIdleFan" type="number" min="0" max="100">
    <div id="presetWrap"><label>材料预设</label><select id="presetSel" onchange="onPresetChange()"></select></div>
    <div class="row" style="margin-top:10px"><button onclick="sendCtrl('start')">启动</button><button onclick="sendCtrl('stop')">停止</button><button onclick="applyCfg()">应用</button></div>
  </div>
  <div class="card span4 panel"><h2>PID</h2>
    <label>Kp</label><input id="setKp" type="number" step="0.001" min="0" max="10">
    <label>Ki</label><input id="setKi" type="number" step="0.0001" min="0" max="1">
    <label>Kd</label><input id="setKd" type="number" step="0.001" min="0" max="20">
    <div class="row" style="margin-top:10px"><button onclick="applyPid()">保存PID</button><button onclick="sendCtrl('pidreset')">恢复默认</button><button onclick="sendCtrl('autotune')">自动校准</button></div>
    <div class="sub" style="margin-top:8px">状态 <span id="tuneState">IDLE</span> | 进度 <span id="tuneProg">0</span>%</div>
  </div>
  <div class="card span4 panel"><h2>Wi-Fi</h2>
    <label>SSID</label><input id="ssid" type="text" placeholder="路由器名称">
    <label>密码</label><input id="pass" type="password" placeholder="路由器密码">
    <button style="margin-top:10px" onclick="saveWifi()">保存并连接</button>
    <div class="sub" style="margin-top:8px">AP模式: Dryer-Config / 12345678 / 192.168.4.1</div>
  </div>
</div>
<script>
const HIS=180,hs={t:[],h:[],f:[],p:[]};
let uiInited=false,lastPresetIndex=-1,forceParamSync=true,forcePidSync=true;
async function jget(u){const r=await fetch(u);return await r.json();}
async function jpost(u,d){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});return await r.json();}
function setText(id,v){document.getElementById(id).textContent=v;}
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
  const rF=calcRange(hs.f,{hardMin:0,hardMax:100,defMin:0,defMax:100,minSpan:10,pad:0.1});
  const rP=calcRange(hs.p,{hardMin:0,hardMax:100,defMin:0,defMax:100,minSpan:10,pad:0.1});
  draw('cTemp',hs.t,'#ef4444',rT.min,rT.max); draw('cHumi',hs.h,'#3b82f6',rH.min,rH.max);
  draw('cFan',hs.f,'#22c55e',rF.min,rF.max); draw('cHeat',hs.p,'#f59e0b',rP.min,rP.max);
}
async function refresh(){
  const s=await jget('/api/status');
  setText('temp',s.temp_c.toFixed(1)); setText('humi',s.humi_pct.toFixed(1)); setText('target',s.target_c.toFixed(1));
  setText('remain',s.remaining_hms); setText('preset',s.preset); setText('active',s.active?'运行中':'已停止');
  setText('heater',s.heater_pct); setText('fan',s.fan_pct);
  document.getElementById('fault').textContent=s.fault===0?'无故障':'故障码: '+s.fault;
  document.getElementById('net').innerHTML=(s.wifi_connected?'<span class="ok">已联网</span> ':'<span class="bad">未联网</span> ')+'IP: '+s.ip+(s.ap_mode?' | AP配网模式':'');
  const presetChanged=(s.preset_index!==lastPresetIndex);
  const allowParamFill=(!uiInited)||forceParamSync||(!s.user_custom_mode&&presetChanged);
  if(allowParamFill){
    if(canFill('setTemp')) document.getElementById('setTemp').value=s.target_c.toFixed(1);
    if(canFill('setDur')) document.getElementById('setDur').value=Math.round(s.config_duration_sec/60);
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
  push(hs.t,s.temp_c); push(hs.h,s.humi_pct); push(hs.f,s.fan_pct); push(hs.p,s.heater_pct); renderCharts();
  lastPresetIndex=s.preset_index; uiInited=true;
}
async function loadPresets(){const p=await jget('/api/presets');const sel=document.getElementById('presetSel');sel.innerHTML='';p.presets.forEach(x=>{const o=document.createElement('option');o.value=x.index;o.textContent=x.name+' ('+x.temp_c+'C)';sel.appendChild(o);});sel.value=p.current_index;}
async function applyCfg(){const mode=document.getElementById('modeSel').value;const payload={target_c:parseFloat(document.getElementById('setTemp').value),duration_min:parseInt(document.getElementById('setDur').value,10),idle_fan_pct:parseInt(document.getElementById('setIdleFan').value,10),use_custom:(mode==='custom')};if(mode==='preset'){payload.preset_index=parseInt(document.getElementById('presetSel').value,10);}await jpost('/api/control',payload);forceParamSync=true;await refresh();}
async function applyPid(){const payload={pid_kp:parseFloat(document.getElementById('setKp').value),pid_ki:parseFloat(document.getElementById('setKi').value),pid_kd:parseFloat(document.getElementById('setKd').value)};await jpost('/api/control',payload);forcePidSync=true;await refresh();}
async function sendCtrl(cmd){await jpost('/api/control',{cmd});if(cmd==='pidreset'){forcePidSync=true;}await refresh();}
async function saveWifi(){await jpost('/api/wifi',{ssid:document.getElementById('ssid').value,password:document.getElementById('pass').value});}
loadPresets().then(refresh);setInterval(refresh,1000);
</script></body></html>)HTML";
  g_webServer.send(200, "text/html; charset=utf-8", PAGE);
}

// 主要功能：输出实时状态 JSON，供网页轮询刷新。
// 使用方法：前端每秒 GET /api/status。
static void handleApiStatus() {
  JsonDocument doc;
  doc["temp_c"] = g_smoothTempC;
  doc["humi_pct"] = g_smoothHumi;
  doc["target_c"] = g_targetTempC;
  doc["config_duration_sec"] = g_configDurationSec;
  doc["remaining_sec"] = g_remainingSec;
  doc["remaining_hms"] = formatDurationForWeb(g_remainingSec);
  doc["preset"] = currentProfileName();
  doc["preset_index"] = static_cast<uint32_t>(g_presetIndex);
  doc["user_custom_mode"] = g_userCustomMode;
  doc["active"] = g_dryingActive;
  doc["heater_pct"] = RatioToPct(g_heaterDemand);
  doc["fan_pct"] = g_fanPct;
  doc["idle_fan_pct"] = g_idleFanPct;
  doc["pid_kp"] = g_pidKp;
  doc["pid_ki"] = g_pidKi;
  doc["pid_kd"] = g_pidKd;
  doc["pid_autotune_msg"] = g_pidAutoTuneMsg;
  uint8_t tuneProgress = 0;
  if (g_pidAutoTuneActive && g_pidAutoTuneState.startMs > 0) {
    uint32_t elapsed = millis() - g_pidAutoTuneState.startMs;
    float pTime = AUTOTUNE_CFG.minTimeMs > 0 ? static_cast<float>(elapsed) / AUTOTUNE_CFG.minTimeMs : 0.0f;
    float pAmp = static_cast<float>(g_pidAutoTuneState.ampCount) / 4.0f;
    float pPeriod = static_cast<float>(g_pidAutoTuneState.periodCount) / 3.0f;
    float p = min(1.0f, min(pTime, min(pAmp, pPeriod)));
    tuneProgress = static_cast<uint8_t>(p * 100.0f);
  } else if (String(g_pidAutoTuneMsg) == "DONE") {
    tuneProgress = 100;
  }
  doc["pid_autotune_progress"] = tuneProgress;
  doc["fault"] = g_faultFlags;
  doc["pid_autotune"] = g_pidAutoTuneActive;
  doc["wifi_connected"] = g_wifiConnected;
  doc["ap_mode"] = g_apConfigMode;
  doc["ip"] = g_wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();

  String out;
  serializeJson(doc, out);
  g_webServer.send(200, "application/json; charset=utf-8", out);
}

// 主要功能：输出预设列表 JSON，供网页下拉框选择。
// 使用方法：前端进入页面时 GET /api/presets。
static void handleApiPresets() {
  JsonDocument doc;
  doc["current_index"] = static_cast<uint32_t>(g_presetIndex);
  JsonArray arr = doc["presets"].to<JsonArray>();
  for (size_t i = 0; i < PRESET_COUNT; ++i) {
    JsonObject p = arr.add<JsonObject>();
    p["index"] = static_cast<uint32_t>(i);
    p["name"] = PRESETS[i].name;
    p["temp_c"] = PRESETS[i].targetTempC;
  }

  String out;
  serializeJson(doc, out);
  g_webServer.send(200, "application/json; charset=utf-8", out);
}

// 主要功能：处理网页控制命令（启停、目标温度、时长、预设等）。
// 使用方法：POST /api/control，JSON 字段支持 cmd/target_c/duration_min/preset_index。
static void handleApiControl() {
  JsonDocument in;
  DeserializationError err = deserializeJson(in, g_webServer.arg("plain"));
  if (err) {
    g_webServer.send(400, "application/json", "{\"ok\":false,\"msg\":\"json格式错误\"}");
    return;
  }

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
    }
  }

  bool hasTarget = (in["target_c"].is<float>() || in["target_c"].is<int>());
  bool hasDuration = in["duration_min"].is<int>();
  bool hasPreset = in["preset_index"].is<int>();
  bool hasMode = in["use_custom"].is<bool>();
  bool useCustom = hasMode ? in["use_custom"].as<bool>() : false;

  if (hasMode) {
    g_userCustomMode = useCustom;
    if (useCustom) {
      hasPreset = false;
    }
  }

  if (hasPreset && !g_userCustomMode) {
    int idx = in["preset_index"].as<int>();
    if (idx >= 0 && static_cast<size_t>(idx) < PRESET_COUNT) {
      applyPreset(static_cast<size_t>(idx));
      if (g_dryingActive) {
        g_remainingSec = g_configDurationSec;
      }
    }
  }

  if (in["target_c"].is<float>() || in["target_c"].is<int>()) {
    g_targetTempC = ClampF(in["target_c"].as<float>(), TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
  }
  if (in["duration_min"].is<int>()) {
    int durMin = in["duration_min"].as<int>();
    uint32_t sec = static_cast<uint32_t>(durMin) * 60UL;
    sec = constrain(sec, DURATION_MIN_SEC, DURATION_MAX_SEC);
    g_configDurationSec = sec;
    if (g_dryingActive) {
      g_remainingSec = g_configDurationSec;
    }
  }
  if (in["idle_fan_pct"].is<int>()) {
    g_idleFanPct = constrain(in["idle_fan_pct"].as<int>(), 0, 100);
  }
  if (!hasMode && !hasPreset && (hasTarget || hasDuration)) {
    g_userCustomMode = true;
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
  g_webServer.send(200, "application/json", "{\"ok\":true}");
}

// 主要功能：处理 Wi-Fi 凭据写入与重连。
// 使用方法：POST /api/wifi，JSON 字段 ssid/password。
static void handleApiWifi() {
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

// 主要功能：404 兜底处理，在 AP 模式下引导到首页。
// 使用方法：WebServer 未命中路由时自动调用。
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
  g_webServer.on("/", HTTP_GET, handleWebRoot);
  g_webServer.on("/api/status", HTTP_GET, handleApiStatus);
  g_webServer.on("/api/presets", HTTP_GET, handleApiPresets);
  g_webServer.on("/api/control", HTTP_POST, handleApiControl);
  g_webServer.on("/api/wifi", HTTP_POST, handleApiWifi);
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
  ledcWrite(FAN_PWM_CHANNEL, duty);
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
  prefs.putUInt("cfgdur", g_configDurationSec);
  prefs.putFloat("kp", g_pidKp);
  prefs.putFloat("ki", g_pidKi);
  prefs.putFloat("kd", g_pidKd);
  prefs.putBool("usermode", g_userCustomMode);
  prefs.putUChar("idlefan", g_idleFanPct);
}

// 主要功能：从 NVS 读取状态并做断电恢复。
// 使用方法：setup 中调用一次。
static void loadState() {
  g_targetTempC = prefs.getFloat("target", PRESETS[0].targetTempC);
  g_configDurationSec = prefs.getUInt("cfgdur", PRESETS[0].durationSec);
  g_pidKp = prefs.getFloat("kp", PID_DEFAULT_KP);
  g_pidKi = prefs.getFloat("ki", PID_DEFAULT_KI);
  g_pidKd = prefs.getFloat("kd", PID_DEFAULT_KD);

  bool lastActive = prefs.getBool("active", false);
  uint32_t lastRemain = prefs.getUInt("remain", 0);
  uint32_t lastPreset = prefs.getUInt("preset", 0);
  g_faultFlags = prefs.getUInt("fault", 0);
  g_userCustomMode = prefs.getBool("usermode", false);
  g_idleFanPct = prefs.getUChar("idlefan", 0);
  g_idleFanPct = constrain(g_idleFanPct, 0, 100);

  if (lastPreset < PRESET_COUNT) g_presetIndex = lastPreset;
  g_targetTempC = ClampF(g_targetTempC, TARGET_TEMP_MIN_C, TARGET_TEMP_MAX_C);
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
static void clearFaults() { g_faultFlags = FAULT_NONE; }

// 主要功能：应用预设（温度、时长、风扇策略索引）。
// 使用方法：菜单/串口选择预设后调用。
static void applyPreset(size_t presetIndex) {
  if (presetIndex >= PRESET_COUNT) return;
  g_presetIndex = presetIndex;
  g_presetGroupFilter = presetGroupOf(presetIndex);
  g_userCustomMode = false;
  g_targetTempC = PRESETS[presetIndex].targetTempC;
  g_configDurationSec = PRESETS[presetIndex].durationSec;
}

// 主要功能：启动烘干任务。
// 使用方法：开始前会清故障并重置 PID 内部状态。
static void startDrying() {
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
  g_dryingActive = false;
  g_heaterDemand = 0.0f;
  setHeater(false);
  Serial.println("已停止烘干。");
  saveState(true);
}

// 主要功能：启动 PID 自动校准（继电振荡法）。
// 使用方法：菜单或串口触发；校准期间会临时接管加热输出。
static void startPidAutoTune() {
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

// 主要功能：处理串口命令（保留调试与远程控制）。
// 使用方法：loop 中周期调用。
static void processSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  String lower = cmd;
  lower.toLowerCase();
  if (lower == "help") {
    Serial.println("命令：help | start | stop | preset <name> | status | faultreset | autotune | wifistatus | wifiap | wificlear");
    return;
  }
  if (lower == "start") {
    startDrying();
    return;
  }
  if (lower == "stop") {
    stopDrying();
    return;
  }
  if (lower.startsWith("preset ")) {
    String name = cmd.substring(7);
    int idx = findPresetByName(name);
    if (idx >= 0) {
      applyPreset(static_cast<size_t>(idx));
      saveState(true);
      Serial.print("已设置预设：");
      Serial.println(activePreset().name);
    } else {
      Serial.println("未知材料预设。");
    }
    return;
  }
  if (lower == "faultreset") {
    clearFaults();
    Serial.println("故障已清除。");
    saveState(true);
    return;
  }
  if (lower == "status") {
    Serial.printf("预设:%s 运行:%u 温度:%.2fC 湿度:%.2f%% 目标:%.1fC 加热:%u%% 风扇:%u%% 故障:%lu 剩余:%lu\n",
                  currentProfileName(), g_dryingActive ? 1 : 0, g_smoothTempC, g_smoothHumi,
                  g_targetTempC, RatioToPct(g_heaterDemand), g_fanPct,
                  static_cast<unsigned long>(g_faultFlags), static_cast<unsigned long>(g_remainingSec));
    return;
  }
  if (lower == "autotune") {
    startPidAutoTune();
    return;
  }
  if (lower == "wifistatus") {
    Serial.printf("WiFi:%s AP:%u IP:%s SSID:%s\n",
                  g_wifiConnected ? "已连接" : "未连接", g_apConfigMode ? 1 : 0,
                  g_wifiConnected ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
                  g_wifiConnected ? WiFi.SSID().c_str() : "(none)");
    return;
  }
  if (lower == "wifiap") {
    startConfigApPortal();
    Serial.println("已切换到 AP 配网模式。");
    return;
  }
  if (lower == "wificlear") {
    clearWifiConfig();
    WiFi.disconnect(true, true);
    startConfigApPortal();
    Serial.println("WiFi 凭据已清除，已进入 AP 配网模式。");
    return;
  }
  Serial.println("未知命令，请输入 help 查看帮助。");
}

// 主要功能：五键菜单交互逻辑。
// 使用方法：loop 中调用，处理菜单切换、温度/PID 参数调整、启停等。
static void processButtonUi() {
  bool up = pollButtonPressed(BTN_UP);
  bool down = pollButtonPressed(BTN_DOWN);
  bool left = pollButtonPressed(BTN_LEFT);
  bool right = pollButtonPressed(BTN_RIGHT);
  bool ok = pollButtonPressed(BTN_OK);

  if (!(up || down || left || right || ok)) return;

  if (g_uiMode == UI_HOME) {
    if (ok) {
      g_uiMode = UI_MENU;
      g_menuIndex = 0;
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
    constexpr uint8_t menuCount = 9;
    if (up && g_menuIndex > 0) g_menuIndex--;
    if (down && g_menuIndex + 1 < menuCount) g_menuIndex++;
    if (left) g_uiMode = UI_HOME;
    if (ok) {
      switch (g_menuIndex) {
        case 0:
          if (g_dryingActive) {
            stopDrying();
          } else {
            startDrying();
          }
          g_uiMode = UI_HOME;
          break;
        case 1:
          g_uiMode = UI_SET_TEMP;
          break;
        case 2:
          g_uiMode = UI_SET_TIME;
          break;
        case 3:
          g_uiMode = UI_SET_IDLE_FAN;
          break;
        case 4:
          g_userCustomMode = true;
          g_uiMode = UI_SET_TEMP;
          break;
        case 5:
          g_uiMode = UI_SET_PRESET;
          break;
        case 6:
          g_uiMode = UI_SET_PID;
          g_pidEditIndex = 0;
          break;
        case 7:
          startPidAutoTune();
          g_uiMode = UI_HOME;
          break;
        case 8:
          clearFaults();
          saveState(true);
          g_uiMode = UI_HOME;
          break;
      }
    }
    return;
  }

  if (g_uiMode == UI_SET_TEMP) {
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
    if (changed) g_userCustomMode = true;
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_TIME) {
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
    if (changed) g_userCustomMode = true;
    if (ok) g_uiMode = UI_MENU;
    saveState(true);
    return;
  }

  if (g_uiMode == UI_SET_IDLE_FAN) {
    if (up) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) + 1, 0, 100);
    if (down) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) - 1, 0, 100);
    if (right) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) + 10, 0, 100);
    if (left) g_idleFanPct = constrain(static_cast<int>(g_idleFanPct) - 10, 0, 100);
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

  if (!g_dryingActive || g_faultFlags != FAULT_NONE || !g_sensorOk) {
    g_heaterDemand = 0.0f;
    return;
  }

  constexpr float DT = CONTROL_INTERVAL_MS / 1000.0f;
  float error = g_targetTempC - g_smoothTempC;

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
    setFanPct(activePreset().fanBasePct);
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

  const MaterialPreset& p = activePreset();
  uint8_t heaterPct = RatioToPct(g_heaterDemand);
  int boosted = p.fanBasePct + static_cast<int>(heaterPct / 2);
  if (g_smoothTempC > g_targetTempC) boosted += 10;
  if (boosted > p.fanMaxPct) boosted = p.fanMaxPct;
  if (boosted < p.fanBasePct) boosted = p.fanBasePct;
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
  if (!g_dryingActive || g_faultFlags != FAULT_NONE || !g_sensorOk) {
    setHeater(false);
    return;
  }

  uint32_t onMs = static_cast<uint32_t>(g_heaterDemand * CONTROL_WINDOW_MS);
  setHeater((now - windowStartMs) < onMs);
}

// 主要功能：根据 UI 模式绘制 OLED。
// 使用方法：当状态变化或到达刷新周期时调用。
static void drawScreen() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);

  if (g_uiMode == UI_HOME) {
    oled.setFont(u8g2_font_wqy12_t_gb2312);
    oled.drawUTF8(0, 11, g_showTempPage ? "温度" : "湿度");
    String ipTop = g_wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    oled.setFont(u8g2_font_4x6_tf);
    oled.drawStr(68, 8, ipTop.c_str());
    oled.drawHLine(0, 14, 128);

    oled.setFont(u8g2_font_logisoso18_tf);
    char mainText[24];
    if (!g_sensorOk) {
      snprintf(mainText, sizeof(mainText), "Err");
    } else if (g_showTempPage) {
      snprintf(mainText, sizeof(mainText), "%.1f C", g_smoothTempC);
    } else {
      snprintf(mainText, sizeof(mainText), "%.1f %%", g_smoothHumi);
    }
    oled.drawStr(0, 38, mainText);

    char line2[48];
    char remain[12];
    FormatDuration(g_remainingSec, remain, sizeof(remain));
    snprintf(line2, sizeof(line2), "P:%s T:%.1fC %s", currentProfileName(), g_targetTempC, remain);
    if (g_userCustomMode) {
      oled.setFont(u8g2_font_wqy12_t_gb2312);
      oled.drawUTF8(0, 50, line2);
      oled.setFont(u8g2_font_5x8_tf);
    } else {
      oled.setFont(u8g2_font_5x8_tf);
      oled.drawStr(0, 50, line2);
    }

    char line3[48];
    if (g_faultFlags != FAULT_NONE) {
      snprintf(line3, sizeof(line3), "FAULT:%lu", static_cast<unsigned long>(g_faultFlags));
    } else if (g_pidAutoTuneActive) {
      snprintf(line3, sizeof(line3), "PID-TUNE %s", g_pidAutoTuneMsg);
    } else if (!g_wifiConnected) {
      if (g_apConfigMode) {
        snprintf(line3, sizeof(line3), "WIFI:AP %s", WiFi.softAPIP().toString().c_str());
      } else {
        snprintf(line3, sizeof(line3), "WIFI:DISCONNECTED");
      }
    } else {
      snprintf(line3, sizeof(line3), "H:%u F:%u %s", RatioToPct(g_heaterDemand), g_fanPct,
               g_dryingActive ? "RUN" : "STOP");
    }
    oled.drawStr(0, 61, line3);
  } else if (g_uiMode == UI_MENU) {
    const char* items[] = {"启动/停止", "目标温度", "烘干时长", "空闲风扇", "自定义参数", "材料预设", "PID参数", "PID自校准", "清除故障"};
    oled.drawUTF8(0, 12, "菜单");
    uint8_t startIdx = (g_menuIndex > 2) ? static_cast<uint8_t>(g_menuIndex - 2) : 0;
    if (startIdx > 5) startIdx = 5;  // 9项菜单显示窗口(4行)
    for (uint8_t i = 0; i < 4; ++i) {
      uint8_t idx = startIdx + i;
      if (idx >= 7) break;
      char row[28];
      snprintf(row, sizeof(row), "%c %s", (idx == g_menuIndex ? '>' : ' '), items[idx]);
      oled.drawUTF8(0, 24 + i * 10, row);
    }
  } else if (g_uiMode == UI_SET_TEMP) {
    oled.drawUTF8(0, 12, "设定目标温度");
    char t[24];
    snprintf(t, sizeof(t), "T=%.1fC", g_targetTempC);
    oled.setFont(u8g2_font_logisoso24_tf);
    oled.drawStr(0, 52, t);
  } else if (g_uiMode == UI_SET_TIME) {
    oled.drawUTF8(0, 12, "设定烘干时长");
    char d[16];
    FormatDuration(g_configDurationSec, d, sizeof(d));
    oled.setFont(u8g2_font_logisoso18_tf);
    oled.drawStr(0, 52, d);
  } else if (g_uiMode == UI_SET_IDLE_FAN) {
    oled.drawUTF8(0, 12, "设定空闲风扇");
    char f[24];
    snprintf(f, sizeof(f), "FAN=%u%%", g_idleFanPct);
    oled.setFont(u8g2_font_logisoso18_tf);
    oled.drawStr(0, 52, f);
  } else if (g_uiMode == UI_SET_PRESET) {
    oled.drawUTF8(0, 12, "选择材料预设");
    oled.setFont(u8g2_font_6x12_tf);
    char gline[24];
    snprintf(gline, sizeof(gline), "组:%s", PRESET_GROUP_NAMES[g_presetGroupFilter]);
    oled.drawUTF8(0, 24, gline);
    oled.setFont(u8g2_font_logisoso18_tf);
    oled.drawStr(0, 46, activePreset().name);
    char p[20];
    snprintf(p, sizeof(p), "T%.1fC", g_targetTempC);
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 56, p);
  } else if (g_uiMode == UI_SET_PID) {
    oled.drawUTF8(0, 12, "PID参数");
    char l1[24];
    char l2[24];
    char l3[24];
    snprintf(l1, sizeof(l1), "%cKp=%.3f", g_pidEditIndex == 0 ? '>' : ' ', g_pidKp);
    snprintf(l2, sizeof(l2), "%cKi=%.3f", g_pidEditIndex == 1 ? '>' : ' ', g_pidKi);
    snprintf(l3, sizeof(l3), "%cKd=%.3f", g_pidEditIndex == 2 ? '>' : ' ', g_pidKd);
    oled.drawStr(0, 28, l1);
    oled.drawStr(0, 40, l2);
    oled.drawStr(0, 52, l3);
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
  if (fsOk) ensureLogFile();

  prefs.begin("dryer", false);
  loadState();
  loadWifiConfig();
  setFanPct(g_idleFanPct);

  g_sensorOk = aht10.begin(&Wire);
  if (!g_sensorOk) {
    setFault(FAULT_SENSOR, "AHT10 初始化失败");
  } else {
    Serial.println("AHT10 初始化成功。");
  }

  if (!connectWifiSta()) {
    startConfigApPortal();
  }
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

  // B. 采样层：读取温湿度，做滤波与故障检测。
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    sensors_event_t humEvent;
    sensors_event_t tempEvent;
    bool ok = aht10.getEvent(&humEvent, &tempEvent);
    if (!ok) {
      if (g_sensorFailCount < 255) g_sensorFailCount++;
      if (g_sensorFailCount >= MAX_SENSOR_FAIL_COUNT) {
        g_sensorOk = false;
        setFault(FAULT_SENSOR, "传感器读取超时");
      }
    } else {
      g_sensorOk = true;
      g_sensorFailCount = 0;
      g_tempC = tempEvent.temperature;
      g_humi = humEvent.relative_humidity;
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
      }
    }
    needRedraw = true;
  }

  // F. 日志层：周期写 SPIFFS + 串口状态。
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    appendLogLine();
    Serial.printf("温度:%.2fC 湿度:%.2f%% 目标:%.1fC 预设:%s 剩余:%lus 加热:%u%% 风扇:%u%% 故障:%lu\n",
                  g_smoothTempC, g_smoothHumi, g_targetTempC, currentProfileName(),
                  static_cast<unsigned long>(g_remainingSec), RatioToPct(g_heaterDemand), g_fanPct,
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
