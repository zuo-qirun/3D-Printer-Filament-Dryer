#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <U8g2lib.h>

constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint8_t OLED_ADDR_8BIT = 0x78;  // 7-bit address 0x3C

Adafruit_AHTX0 aht10;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

constexpr uint32_t SENSOR_INTERVAL_MS = 100;   // 10Hz
constexpr uint32_t DISPLAY_INTERVAL_MS = 50;   // 20Hz
constexpr uint32_t PAGE_INTERVAL_MS = 3000;    // 3 seconds per page
constexpr float SMOOTH_ALPHA = 0.20f;          // exponential smoothing factor
static void drawLargePage(bool showTemperature, float temperatureC, float humidity, bool sensorOk) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);
  oled.drawStr(0, 12, showTemperature ? "Temperature" : "Humidity");

  oled.setFont(u8g2_font_logisoso24_tf);
  char text[24];
  if (!sensorOk) {
    snprintf(text, sizeof(text), "Err");
  } else if (showTemperature) {
    snprintf(text, sizeof(text), "%.1f C", temperatureC);
  } else {
    snprintf(text, sizeof(text), "%.1f %%", humidity);
  }
  oled.drawStr(0, 56, text);
  oled.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  oled.setI2CAddress(OLED_ADDR_8BIT);
  oled.begin();
  oled.setContrast(255);

  bool ok = aht10.begin(&Wire);
  Serial.println(ok ? "AHT10 init OK (0x38)." : "AHT10 init failed.");
  drawLargePage(true, 0.0f, 0.0f, ok);
}

void loop() {
  static uint32_t lastSensorMs = 0;
  static uint32_t lastDisplayMs = 0;
  static uint32_t lastPageMs = 0;
  static uint32_t lastLogMs = 0;
  static float temperatureC = 0.0f;
  static float humidity = 0.0f;
  static float smoothTemperatureC = 0.0f;
  static float smoothHumidity = 0.0f;
  static bool smoothInited = false;
  static bool ok = false;
  static bool showTemperature = true;

  uint32_t now = millis();
  bool needRedraw = false;

  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    sensors_event_t humidityEvent;
    sensors_event_t tempEvent;
    ok = aht10.getEvent(&humidityEvent, &tempEvent);

    if (ok) {
      temperatureC = tempEvent.temperature;
      humidity = humidityEvent.relative_humidity;

      if (!smoothInited) {
        smoothTemperatureC = temperatureC;
        smoothHumidity = humidity;
        smoothInited = true;
      } else {
        smoothTemperatureC += SMOOTH_ALPHA * (temperatureC - smoothTemperatureC);
        smoothHumidity += SMOOTH_ALPHA * (humidity - smoothHumidity);
      }
    } else {
      Serial.println("AHT10 read failed.");
    }
  }

  if (now - lastLogMs >= 500 && ok) {
    lastLogMs = now;
    Serial.print("Temperature: ");
    Serial.print(smoothTemperatureC, 2);
    Serial.print(" C, Humidity: ");
    Serial.print(smoothHumidity, 2);
    Serial.println(" %");
  }

  if (now - lastPageMs >= PAGE_INTERVAL_MS) {
    lastPageMs = now;
    showTemperature = !showTemperature;
    needRedraw = true;
  }

  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    needRedraw = true;
  }

  if (needRedraw) {
    drawLargePage(showTemperature, smoothTemperatureC, smoothHumidity, ok);
  }

  delay(10);
}
