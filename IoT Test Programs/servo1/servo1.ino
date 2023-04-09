#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal.h>

#define SERVO_PIN 12 // ESP32 pin GIOP12 connected to servo motor

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21,1, 3, 17, 16);

Servo servoMotor;

void setup() {
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
  delay(1000);
  lcd.print ("Servo Motor Test");
}

void loop() {
  // rotates from 0 degrees to 180 degrees
  for (int pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    servoMotor.write(pos);
    delay(15); // waits 15ms to reach the position
  }

  // rotates from 180 degrees to 0 degrees
  for (int pos = 180; pos >= 0; pos -= 1) {
    servoMotor.write(pos);
    delay(15); // waits 15ms to reach the position
  }
}
