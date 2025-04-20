#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define MOISTURE_PIN A0
#define BATTERY_CONTROL_PIN A5
#define BATTERY_ADC_PIN A1
#define PUMP_PIN 3

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

const float MOISTURE_THRESHOLD = 40.0; // % threshold to start watering
const int WATERING_TIME_MS = 3000;     // 3 seconds

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(BATTERY_CONTROL_PIN, OUTPUT);
  digitalWrite(BATTERY_CONTROL_PIN, LOW); // disable divider initially

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!aht.begin()) {
    Serial.println("❌ AHT20 not found!");
  } else {
    Serial.println("✅ AHT20 ready.");
  }

  if (!bmp.begin(0x77)) {
    Serial.println("❌ BMP280 not found!");
  } else {
    Serial.println("✅ BMP280 ready.");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   // temp
                    Adafruit_BMP280::SAMPLING_X16,  // pressure
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

void loop() {
  // --- Sensor Readings ---
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float pressure = bmp.readPressure() / 100.0F;

  int moistureADC = analogRead(MOISTURE_PIN);
  float moisturePercent = map(moistureADC, 3000, 1300, 0, 100); // adjust if needed
  moisturePercent = constrain(moisturePercent, 0, 100);

  // --- Battery Voltage Read ---
  digitalWrite(BATTERY_CONTROL_PIN, HIGH);
  delay(10); // allow divider to settle
  int batteryADC = analogRead(BATTERY_ADC_PIN);
  digitalWrite(BATTERY_CONTROL_PIN, LOW);

  float voltage = batteryADC / 4095.0 * 3.3 * ((47.0 + 47.0) / 47.0); // adjust for divider

  // --- Output ---
  Serial.printf("🌡 Temp: %.2f °C | 💧 Humidity: %.2f %% | ⬇️ Pressure: %.2f hPa\n", temp.temperature, humidity.relative_humidity, pressure);
  Serial.printf("🌱 Soil Moisture: %.1f %% (raw: %d)\n", moisturePercent, moistureADC);
  Serial.printf("🔋 Battery Voltage: %.2f V (ADC: %d)\n", voltage, batteryADC);

  // --- Watering Logic ---
  if (moisturePercent < MOISTURE_THRESHOLD) {
    Serial.println("⚠️ Soil is dry. Starting pump...");
    digitalWrite(PUMP_PIN, HIGH);
    delay(WATERING_TIME_MS);
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("✅ Watering done.");
  } else {
    Serial.println("👌 Soil moisture OK. No watering needed.");
  }

  delay(10000); // wait 10 seconds before next cycle (adjust for testing)
}
