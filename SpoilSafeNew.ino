#include <ArduinoBLE.h>
#include <Wire.h>
#include "bytearray.hpp"
#include "i2c_helper.hpp"

#define LCD_ADDR ((byte)0x72)
#define LED_BRIGHTNESS ((byte)0x19)
#define MEM_ADDR ((byte)0x50)

const int gasSensorPin = A0;
const int ledPin = D5;
const int redLedPin = D8;
const int threshold = 840;

static byte mem_pos = 0;

// Global variables to hold the elapsed rot time
int globalElapsedHours = 0;
int globalElapsedMinutes = 0;
int globalElapsedSeconds = 0;

// Initialize BLE Service and Characteristic
BLEService gasSensorService("12345678-1234-5678-1234-56789abcdef0");
BLEStringCharacteristic gasDataCharacteristic(
  "12345678-1234-5678-1234-56789abcdef1",
  BLERead | BLENotify,
  50  // max length
);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // Start BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1)
      ;
  }

  BLE.setLocalName("NanoESP32-GasSensor");
  BLE.setAdvertisedService(gasSensorService);
  gasSensorService.addCharacteristic(gasDataCharacteristic);
  BLE.addService(gasSensorService);
  BLE.advertise();

  Serial.println("BLE Ready, waiting for connection...");

  delay(5000);
}

void loop() {
  BLEDevice central = BLE.central();

  // First call to update the gas reading
  int currentGas = updateGases();
  Serial.println(currentGas);
  Serial.print(": Current gas reading");

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      int currentGas = updateGases();
      double t = calculateTime(currentGas);

      // Build BLE data string including elapsed time
      char bleData[60];
      snprintf(
        bleData,
        sizeof(bleData),
        "Gas: %d | Hours: %.2f | Elapsed: %02d:%02d:%02d",
        currentGas,
        t,
        globalElapsedHours,
        globalElapsedMinutes,
        globalElapsedSeconds);

      gasDataCharacteristic.writeValue(bleData);
      Serial.print("Data sent over BLE: ");
      Serial.println(bleData);
    }
    Serial.println("Disconnected from BLE");
  }
}

// Function to set backlight color for the LCD
void setBacklightColor(byte red, byte green, byte blue) {
  bytearray color = bytearray(5);
  color[0] = 0x7C;
  color[1] = 0x2B;
  color[2] = red;
  color[3] = green;
  color[4] = blue;
  i2c_writeTo(LCD_ADDR, color);
}

int updateGases() {

  int currentGas = analogRead(gasSensorPin);
  currentGas = currentGas;
  Serial.println(currentGas);

  if (currentGas < 5) {
    currentGas = 1;
  }

  Serial.println("Current Gas: ");
  Serial.print(currentGas);


  double t = calculateTime(currentGas);
  Serial.print("Projected Hours Until Rotted: ");
  Serial.println(t, 4);

  // Visual feedback with LED
  if (t < 15) {
    digitalWrite(ledPin, HIGH);
    digitalWrite(redLedPin, LOW);
  } else {
    digitalWrite(ledPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }

  // Update LCD display
  setBacklightColor(0, 255, 0);  // Green backlight

  if (t <= 10) setBacklightColor(255, 0, 0);
  else if ((t > 10) && (t <= 20)) setBacklightColor(255, 255, 0);
  else setBacklightColor(0, 255, 0);

  int dispAddr[2] = { 0x7C, 0x2D };
  bytearray clearDisplay = bytearray(dispAddr, 2);
  i2c_writeTo(LCD_ADDR, clearDisplay);
  
  char str[32];
  snprintf(str, sizeof(str), "   Fresh for:     %.2f Hours", t);
  i2c_writeTo(LCD_ADDR, bytearray((str)));

  // Calculate total time since start
  unsigned long timeSinceStart = millis();
  double minutes = timeSinceStart / 60000.0;
  int hours = minutes / 60;
  int minutes2 = (int)minutes % 60;
  int seconds = (timeSinceStart - (hours * 3600000) - (minutes2 * 60000)) / 1000;

  // Memory logic
  if ((currentGas < memRead(0, mem_pos + 3)) || (t > 1.25 * (memRead(0, mem_pos + 4)))) {
    memWrite(0, mem_pos + 3, currentGas);  //lowest gas into mem 3
    memWrite(0, mem_pos, seconds);
    memWrite(0, mem_pos + 1, minutes2);
    memWrite(0, mem_pos + 2, hours);
    Serial.println("Memory updated.");
  }

  memWrite(0, mem_pos + 4, t);  // storing time into memory

  long storedSeconds = (3600 * memRead(0, mem_pos + 2)) + (60 * memRead(0, mem_pos + 1)) + memRead(0, mem_pos);
  long currentSeconds = timeSinceStart / 1000;
  long difference = currentSeconds - storedSeconds;
  if (difference < 0) difference = 0;

  int newHours = difference / 3600;
  int newMinutes = (difference - (newHours * 3600)) / 60;
  int newSeconds = difference - (newHours * 3600) - (newMinutes * 60);

  // Store the elapsed time in global variables so we can access them in loop()
  globalElapsedHours = newHours;
  globalElapsedMinutes = newMinutes;
  globalElapsedSeconds = newSeconds;

  delay(2848);

  i2c_writeTo(LCD_ADDR, clearDisplay);  // clears the display
  snprintf(str, sizeof(str), "  Elapsed Rot:      %02d:%02d:%02d", newHours, newMinutes, newSeconds);
  i2c_writeTo(LCD_ADDR, bytearray((str)));

  delay(2848);

  return currentGas;
}

double calculateTime(int currentGas) {
  double k = -0.05;
  double logInside = (double)currentGas / threshold;
  double t = log(logInside) / k;

  if (t<0) {
    t=0;
  }

  return t;
}

// Writes data to an address in the EEPROM
void memWrite(int high_addr, int low_addr, int data) {
  bytearray memData(3);

  memData[0] = (byte)high_addr;
  memData[1] = (byte)low_addr;
  memData[2] = (byte)data;

  i2c_writeTo(MEM_ADDR, memData);
  delay(6);
}

byte memRead(int high_addr, int low_addr) {
  bytearray memoryData(2);

  memoryData[0] = (byte)high_addr;
  memoryData[1] = (byte)low_addr;

  i2c_writeTo(MEM_ADDR, memoryData);
  bytearray mem = i2c_readFrom(MEM_ADDR, 1);

  return mem[0];
}
