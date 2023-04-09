#include <Stepper.h>
#include <Wire.h> 
#include <LiquidCrystal.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21,1, 3, 17, 16);

// L293D Motor Driver Pins
#define IN1 13
#define IN2 26
#define IN3 27
#define IN4 33

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void setup() {
  // set the speed at 5 rpm
  myStepper.setSpeed(5);
  // initialize the serial port
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();
  delay(1000);
  lcd.print ("Stepper Motor Test");
  delay(2000);
  lcd.clear();
}

void loop() {
  // step one revolution in one direction:
  Serial.println("counterclockwise");
  lcd.print ("Counterclockwise");
  myStepper.step(stepsPerRevolution);
  delay(1000);
  lcd.clear();

  // step one revolution in the other direction:
  Serial.println("Clockwise");
  lcd.print("Clockwise");
  myStepper.step(-stepsPerRevolution);
  delay(1000);
  lcd.clear();
}
