#include <Wire.h> 
#include <LiquidCrystal.h>

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21,1, 3, 17, 16);

const int buzzer = 15;

void setup() {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
  delay(1000);
  lcd.print ("Buzzer Test");
  delay(2000);
  lcd.clear();
}

void loop() {
  lcd.print ("Buzzer ON");
  tone(buzzer, 1000);
  delay(1000);
  lcd.clear();
  lcd.print ("Buzzer OFF");
  noTone(buzzer);
  delay(1000);
  lcd.clear();
}
