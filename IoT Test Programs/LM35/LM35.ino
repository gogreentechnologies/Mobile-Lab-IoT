#include <Wire.h> 
#include <LiquidCrystal.h>

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       34 // ESP32 pin GIOP39 (ADC0) connected to LM35

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21, 1, 3, 17, 16);

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");

  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
}

void loop() {
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 32;

  // print the temperature in the Serial Monitor:
  Serial.print("Temperature: ");
  lcd.print("Temperature :");
  Serial.print(tempC);   // print the temperature in °C
  Serial.print("°C");
  lcd.setCursor(0,1);
  lcd.print(tempC);   // print the temperature in °C
  lcd.print("C");
  Serial.print("  ~  ");// separator between °C and °F
  lcd.print(" ~ ");
  Serial.print(tempF);   // print the temperature in °F
  Serial.println("°F");
  lcd.print(tempF);   // print the temperature in °F
  lcd.println("F ");
  delay(500);
}
