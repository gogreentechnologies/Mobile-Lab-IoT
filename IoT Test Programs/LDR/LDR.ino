#include <Wire.h> 
#include <LiquidCrystal.h>

#define LIGHT_SENSOR_PIN 36 // ESP32 pin GIOP36 (ADC0)

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21, 1, 3, 17, 16);

void setup() {
  // initialize serial communication at 112500 bits per second:
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
  lcd.print ("LDR Sensor Test");
  delay(2000);
  lcd.clear();
}

void loop() {
  // reads the input on analog pin (value between 0 and 4095)
  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  Serial.print("Analog Value = ");
  lcd.print("LDR Value :");
  Serial.print(analogValue);   // the raw analog reading
  lcd.print(analogValue);

  delay(500);
  lcd.clear();
}
