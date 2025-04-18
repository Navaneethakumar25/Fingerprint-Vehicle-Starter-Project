
/*
 * Fingerprint Vehicle Starter System with LCD, Motor, Push Buttons and Atmega328
 * Components: Fingerprint module (e.g., R305), LCD (16x2), Push Buttons, DC Motor, Motor Driver (L298N), Atmega328
 * Software: Arduino IDE (using embedded C-style syntax)
 */

#include <LiquidCrystal.h>
#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>

// Fingerprint sensor serial (using pins 2 and 3)
SoftwareSerial fingerSerial(2, 3);
Adafruit_Fingerprint finger(&fingerSerial);

// LCD pins: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

// Motor driver pins
#define MOTOR_IN1 5
#define MOTOR_IN2 6

// Push button pins
#define START_BUTTON A0
#define STOP_BUTTON A1

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  lcd.begin(16, 2);
  lcd.print("Vehicle Starter");

  Serial.begin(9600);
  finger.begin(57600);

  if (finger.verifyPassword()) {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Ready");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Error");
    while (1); // Halt
  }

  delay(2000);
  lcd.clear();
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print("Place Finger...");

  if (getFingerprintID()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Access Granted");

    // Wait for start button
    while (digitalRead(START_BUTTON) == HIGH);

    startMotor();

    // Wait for stop button
    lcd.setCursor(0, 1);
    lcd.print("Running...");

    while (digitalRead(STOP_BUTTON) == HIGH);

    stopMotor();
    lcd.clear();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Access Denied");
    delay(2000);
    lcd.clear();
  }
}

bool getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return false;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return false;

  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.print("Found ID #"); Serial.println(finger.fingerID);
    return true;
  } else {
    return false;
  }
}

void startMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}
