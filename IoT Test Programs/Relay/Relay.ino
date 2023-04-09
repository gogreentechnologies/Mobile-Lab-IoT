#include <Wire.h> 
#include <LiquidCrystal.h>

#define PIN_RELAY  14 // the ESP32 pin, which connects to the relay

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21,1, 3, 17, 16);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAY, OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
  delay(1000);
  lcd.print ("DC Motor Test");
  delay(2000);
  lcd.clear();
}

void loop() {
  lcd.print ("DC Motor ON");
  digitalWrite(PIN_RELAY, 1);
  delay(5000);
  lcd.clear();
  lcd.print ("DC Motor OFF");
  digitalWrite(PIN_RELAY, 0);
  delay(5000);
  lcd.clear();
}
