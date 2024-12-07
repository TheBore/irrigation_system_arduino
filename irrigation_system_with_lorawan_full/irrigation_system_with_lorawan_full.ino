#include <Wire.h>
#include "rgb_lcd.h"
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "TheThingsNetwork.h"
#include "CayenneLPP.h"

// Sensor and hardware definitions
#define DHTPIN 7          // Pin connected to the DHT sensor data pin
#define RELAY_PIN 3       // Pin connected to the relay
#define DHTTYPE DHT11     // DHT 11
#define CAPACITIVE_PIN 5  // Capacitive moisture sensor
#define BUTTON 8          // Example button pin (not used for LoRa)

// LoRa configurations
#define loraSerial Serial2
#define debugSerial SerialUSB
#define SERIAL_TIMEOUT 10000
#define freqPlan TTN_FP_EU868
#define SF 9
#define FSB 0
#define LORA_INTERVAL 10000  // 10 seconds

// LoRa OTAA credentials
const char *joinEui = "0000000000000000";
const char *appKey = "490F403E4561996DC8CFB50CC9FE0EDC";

// Timer for 10-second interval
unsigned long lastTransmissionTime = 0;

enum state {WHITE_COLOR, RED_COLOR, GREEN_COLOR, BLUE_COLOR, CYAN_COLOR, ORANGE_COLOR, PURPLE_COLOR, OFF};    // List of colours for the RGB LED

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
  {2, false, true}, // Red button
  {12, false, true}
};

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
rgb_lcd lcd;

TheThingsNetwork ExpLoRer(loraSerial, debugSerial, freqPlan, SF, FSB);
CayenneLPP CayenneRecord(51);

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.begin(16, 2);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  LED(RED_COLOR);
  analogReadResolution(10);  

  pinMode(LORA_RESET, OUTPUT);          //Reset the LoRa module to a known state
  digitalWrite(LORA_RESET, LOW);
  delay(100);
  digitalWrite(LORA_RESET, HIGH);
  delay(1000);                          // Wait for RN2483 to reset
  LED(ORANGE_COLOR);                          // Switch to ORANGE after reset

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

  debugSerial.begin(57600);
  loraSerial.begin(57600);

  // Wait for serial monitor
  while (!debugSerial && millis() < SERIAL_TIMEOUT);

  // Set callback for incoming messages
  ExpLoRer.onMessage(message);

  // Set up LoRa communications
  debugSerial.println("-- STATUS");
  ExpLoRer.showStatus();

  delay(1000);

  // For OTAA
  debugSerial.println("-- JOIN");
  if (ExpLoRer.join(joinEui, appKey, 6, 3000)) { // 6 Re-tries, 3000ms delay in between
    debugSerial.println("LoRa joined successfully");
    displayLoraMessage(true);
    LED(GREEN);               // Switch to GREEN if OTAA join is successful
  } else {
    debugSerial.println("LoRa join failed");
    displayLoraMessage(false);
    LED(RED);                 // Switch to RED if OTAA join fails
  }
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

  // Handle regular data transmission
  if (millis() - lastTransmissionTime >= LORA_INTERVAL) {
    sendLoRaData();
    lastTransmissionTime = millis();
  }
}

void handleButtonPress(int buttonIndex) {
  lcd.clear();
  switch (buttonIndex) {
    case 0: // Green button
      displayTemperatureAndHumidity();
      break;
    case 1: // Yellow button
      displayPressureAndAltitude();
      break;
    // Add cases for other buttons
    case 2: // Black button
      displayMoisture();
      break;
    case 3: // Red button
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

void displayLoraMessage(bool successful) {
  if (successful) {
    lcd.setRGB(0, 0, 195);
    lcd.setCursor(0, 0);
    lcd.print("LoRa success!");
  } else {
    lcd.setRGB(195, 0, 0);
    lcd.setCursor(0, 0);
    lcd.print("LoRa failed!");
  }
}

void sendLoRaData() {
  // Collect sensor data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);
  int capMoistureValue = analogRead(CAPACITIVE_PIN);

  // Add data to Cayenne payload
  CayenneRecord.reset();
  CayenneRecord.addGPS(1, 41.9881673, 21.4694947, 180);
  CayenneRecord.addTemperature(1, temperature);
  CayenneRecord.addRelativeHumidity(2, humidity);
  CayenneRecord.addBarometricPressure(3, pressure);
  CayenneRecord.addAnalogInput(4, capMoistureValue);

  // Send data via LoRa
  LED(BLUE_COLOR); 
  byte payloadSize = CayenneRecord.getSize();
  byte response = ExpLoRer.sendBytes(CayenneRecord.getBuffer(), payloadSize, 1, false);

  LED(response +1);                                   // Change LED colour depending on module response to uplink success
  delay(100);

  CayenneRecord.reset();  // Clear the record buffer for the next loop
  LED(OFF);

  if (response == 1) {
    displayLoraMessage(true);
  }
  else {
    displayLoraMessage(false);
  }

}

void LED(byte state)
{
  switch (state)
  {
  case WHITE_COLOR:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW); 
    break;
  case RED_COLOR:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH); 
    break;
  case ORANGE_COLOR:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH); 
    break;
  case CYAN_COLOR:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW); 
    break;
  case PURPLE_COLOR:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    break;
  case BLUE_COLOR:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    break;
  case GREEN_COLOR:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    break;
  default:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    break;
  }
}

// Callback method for receiving downlink messages. Uses ExpLoRer.onMessage(message) in setup()
void message(const uint8_t *payload, size_t size, port_t port)
{
  debugSerial.println("-- MESSAGE");
  debugSerial.print("Received " + String(size) + " bytes on port " + String(port) + ":");

  for (int i = 0; i < size; i++)
  {
    debugSerial.print(" " + String(payload[i]));
  }

  debugSerial.println();
}