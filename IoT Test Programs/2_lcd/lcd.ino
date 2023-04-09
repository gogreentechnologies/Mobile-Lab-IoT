#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>

// Define the round() function if it's not already available

long round(double x) {
  if (x >= 0) {
    return (long)(x + 0.5);
  } else {
    return (long)(x - 0.5);
  }
}

// Create an LCD object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21, 1, 3, 17, 16);

void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");

  // Go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0, 1);
  lcd.print("IoT Mobile Lab");
}

void loop() {}
