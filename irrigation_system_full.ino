#include <Wire.h>
#include "rgb_lcd.h"
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// There are special settings because the testing tree is evergreen

// Temperature and humidity
#define DHTPIN 7 // Pin connected to the DHT sensor data pin
#define RELAY_PIN 3 // Pin connected to the relay
#define DHTTYPE DHT11 // DHT 11
#define CAPACITIVE_PIN 5 // This is the capacitive sensor

// Button and sensor configuration
struct Button {
  int pin;
  bool currentState;
  bool lastState;
};

Button buttons[] = {
  {8, false, true}, // Green button
  {6, false, true}, // Black button
  {4, false, true}, // Yellow buton
  {2, false, true}, // Green button
  {12, false, true}
};

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
rgb_lcd lcd;

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.begin(16, 2);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Initialize button pins
  for (int i = 0; i < 5; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }

  if (!bmp.begin(0x77)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
  // Loop through all buttons
  for (int i = 0; i < 5; i++) {
    buttons[i].currentState = digitalRead(buttons[i].pin) == LOW;
    Serial.println(buttons[i].currentState);

    if (buttons[i].currentState && !buttons[i].lastState) {
      // Handle button press based on index
      handleButtonPress(i);
      delay(500); // Debounce delay
    }

    buttons[i].lastState = buttons[i].currentState;
  }
}

void handleButtonPress(int buttonIndex) {
  lcd.clear();
  switch (buttonIndex) {
    case 0: // Green button
      displayTemperatureAndHumidity();
      break;
    case 1: // Orange button
      displayPressureAndAltitude();
      break;
    // Add cases for other buttons
    case 2:
      displayMoisture();
      break;
    case 3:
      // Additional button actions
      pump();
      break;
    case 4:
      // Additional button actions
      break;
  }
}

void displayTemperatureAndHumidity() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  lcd.setRGB(0, 255, 0);
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.write(223); // ASCII value for the degree symbol
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humid: ");
  lcd.print(humidity);
  lcd.print("%");
}

void displayPressureAndAltitude() {
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);
  lcd.setRGB(255, 228, 181);
  lcd.setCursor(0, 0);
  lcd.print("Press: ");
  lcd.print(pressure);
  lcd.print("hPa");
  lcd.setCursor(0, 1);
  lcd.print("Alt: ");
  lcd.print(altitude);
  lcd.print("m");
}

void displayMoisture() {
  int capMoistureValue = analogRead(CAPACITIVE_PIN);
  lcd.setRGB(255, 255, 0);
  lcd.setCursor(0, 0);
  lcd.print("Moisture: ");
  lcd.setCursor(0, 1);
  lcd.print(capMoistureValue);
}

void pump() {
  Serial.println("Turning pump ON");
  lcd.setRGB(255, 0, 0);
  lcd.setCursor(0, 0);
  lcd.print("Pump is turned");
  lcd.setCursor(0, 1);
  lcd.print("on for 5s");
  digitalWrite(RELAY_PIN, HIGH);
  delay(5000);
  Serial.println("Turning pump OFF");
  digitalWrite(RELAY_PIN, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pump finished");
}
