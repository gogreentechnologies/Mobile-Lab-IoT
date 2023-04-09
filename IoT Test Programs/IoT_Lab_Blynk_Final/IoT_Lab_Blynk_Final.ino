#include <Wire.h> 
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Servo.h>
#include <AD9833.h>     // Include the library
#include <cmath>

// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLMDtc1qei"
#define BLYNK_DEVICE_NAME "DEMO"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define FNC_PIN 5       // Can be any digital IO pin

//--------------- Create an AD9833 object ----------------
// Note, SCK and MOSI must be connected to CLK and DAT pins on the AD9833 for SPI
AD9833 gen(FNC_PIN);       // Defaults to 25MHz internal reference frequency

#define ADC_VREF_mV    3300.0 // in millivolt
#define APP_DEBUG


#define Frequency 1000      // Frequency = 1Hz
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       34 // ESP32 pin GIOP39 (ADC0) connected to LM35

#define echoPin1 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin1 4 //attach pin D3 Arduino to pin Trig of HC-SR04

#define VPIN_Servo V2
#define VPIN_Buzzer V0
#define VPIN_DCMOTOR V1
#define VPIN_LM35 V5
#define VPIN_Stepper V3
#define VPIN_LDR V4
#define VPIN_Sine V10
#define VPIN_Triangle V11
#define VPIN_Square V12
#define VPIN_Ultrasonic V16

#define PIN_RELAY  14 // the ESP32 pin, which connects to the relay
#define LIGHT_SENSOR_PIN 36 // ESP32 pin GIOP36 (ADC0)
#define SERVO_PIN 12 // ESP32 pin GIOP18 connected to servo motor


// L293D Motor Driver Pins
#define IN1 13
#define IN2 26
#define IN3 27
#define IN4 33


// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21,1, 3, 17, 16);

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
int pos = 0;    // variable to store the servo position

// defines variables
long duration1; // variable for the duration of sound wave travel
int distance1; // variable for the distance measurement

//WidgetLED led1(V15);

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

const int buzzer = 15;
Servo servoMotor;

bool toggleState_buzzer = LOW;
bool toggleState_dcmotor = LOW;
bool toggleState_lm35 = LOW;
bool toggleState_Stepper = LOW;
bool toggleState_Servo = LOW;
bool toggleState_ldr = LOW;
bool toggleState_Sine = LOW;
bool toggleState_Triangle = LOW;
bool toggleState_Square = LOW;
bool toggleState_ultrasonic = LOW;

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT

#include "BlynkEdgent.h"

BLYNK_CONNECTED()
{
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_Buzzer);
  Blynk.syncVirtual(VPIN_DCMOTOR);
  Blynk.syncVirtual(VPIN_LM35);
  Blynk.syncVirtual(VPIN_Stepper);
  Blynk.syncVirtual(VPIN_Servo);
  Blynk.syncVirtual(VPIN_Sine);
  Blynk.syncVirtual(VPIN_Triangle);
  Blynk.syncVirtual(VPIN_Square);
  Blynk.syncVirtual(VPIN_Ultrasonic);
}

BLYNK_WRITE(VPIN_Servo)
{
  toggleState_Servo = param.asInt();
  if(toggleState_Servo == 1){
    lcd.print ("Servo Motor On");
    servomotor();
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_Buzzer)
{
  toggleState_buzzer = param.asInt(); //requests for latest state of the button
  if(toggleState_buzzer == 1){
    for (int i=5; i>0; i--){
      lcd.print ("Buzzer ON");
      tone(buzzer, 1000);
      delay(1000);
      noTone(buzzer);
      delay(1000);
      Serial.println(i);
      lcd.clear();
    }  
    lcd.clear();
  }
  else{
    noTone(buzzer);
  }
}

BLYNK_WRITE(VPIN_DCMOTOR)
{
  toggleState_dcmotor = param.asInt();
  if(toggleState_dcmotor == 1){
    lcd.print ("DC Motor ON");
    digitalWrite(PIN_RELAY, 1);
  }
  else{
    digitalWrite(PIN_RELAY, 0);
    lcd.clear();
  }
}


BLYNK_WRITE(VPIN_LM35)
{
  toggleState_lm35 = param.asInt();
  if(toggleState_lm35 == 1){
      // read the ADC value from the temperature sensor
      int adcVal = analogRead(PIN_LM35);
      // convert the ADC value to voltage in millivolt
      float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
      // convert the voltage to the temperature in °C
      float tempC = milliVolt / 10;
      // convert the °C to °F
      float tempF = tempC * 9 / 5 + 32;
      // print the temperature in the Serial Monitor:
      lcd.print("Temperature :");
      lcd.setCursor(0,1);
      lcd.print(tempC);   // print the temperature in °C
      lcd.print("C");
      lcd.print(" ~ ");
      lcd.print(tempF);   // print the temperature in °F
      lcd.println("F ");
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_Stepper)
{
  toggleState_Stepper = param.asInt();
  if(toggleState_Stepper == 1){
    lcd.print ("Stepper Motor On");
    steppermotor();
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_LDR)
{
  toggleState_ldr = param.asInt();
  if(toggleState_ldr == 1){
      // reads the input on analog pin (value between 0 and 4095)
      int analogValue = analogRead(LIGHT_SENSOR_PIN);

      Serial.print("Analog Value = ");
      lcd.print("LDR Value :");
      Serial.print(analogValue);   // the raw analog reading
      lcd.print(analogValue);
      Blynk.virtualWrite(V8, analogValue);
              }
              else{
                lcd.clear();
                }
}

BLYNK_WRITE(VPIN_Sine)
{
  toggleState_Sine = param.asInt();
  if(toggleState_Sine == 1){
    lcd.print ("Sine Wave");
    sinewave();
    Serial.println(analogRead(39));
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_Triangle)
{
  toggleState_Triangle = param.asInt();
  if(toggleState_Triangle == 1){
    lcd.print ("Triangular Wave");
    trianglewave();
    Serial.println(analogRead(39));
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_Square)
{
  toggleState_Square = param.asInt();
  if(toggleState_Square == 1){
    lcd.print ("Square Wave");
    squarewave();
    Serial.println(analogRead(39));
  }
  else{
    lcd.clear();
  }
}

BLYNK_WRITE(VPIN_Ultrasonic)
{
  toggleState_ultrasonic = param.asInt();
  if(toggleState_ultrasonic == 1){
     // Clears the trigPin condition
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance1 = duration1 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  lcd.println("Ultrasonic");
  lcd.setCursor(0,1);
  lcd.println("Sensor: ");
  Serial.print(distance1);
  lcd.println("Sensor: ");
  //lcd.print(distance1);
  Serial.println(" cm");
  //lcd.println("cm");
  }
  else{
    lcd.clear();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Welcome to SPIT");
  // go to row 1 column 0, note that this is indexed at 0
  lcd.setCursor(0,1); 
  lcd.print ("IoT Mobile Lab");
  delay(2000);
  lcd.clear();

  pinMode(buzzer, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  // set the speed at 5 rpm
  myStepper.setSpeed(5);
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  BlynkEdgent.begin();
}

void loop() {
  temperature();
  void ldr();
  
  BlynkEdgent.run();
  ultrasonic();
}

void servomotor(){
  for (int i=3; i>0; i--){
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
    Serial.println(i);
  }
}

void temperature(){
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 32;
  delay(500);

  Blynk.virtualWrite(V6, tempC);
  Blynk.virtualWrite(V7, tempF);
}

void ldr() {
  // reads the input on analog pin (value between 0 and 4095)
  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  Serial.print("Analog Value = ");
  lcd.print("LDR Value :");
  Serial.print(analogValue);   // the raw analog reading
  lcd.print(analogValue);
  Blynk.virtualWrite(V8, analogValue);

  delay(500);
  lcd.clear();
}

void steppermotor(){
  for (int i=3; i>0; i--){
    // step one revolution in one direction:
    Serial.println("counterclockwise");
    lcd.setCursor(0,1); 
    lcd.print ("Counterclockwise");
    myStepper.step(stepsPerRevolution);
    delay(1000);
    lcd.clear();

    // step one revolution in the other direction:
    Serial.println("Clockwise");
    lcd.setCursor(0,1); 
    lcd.print("Clockwise");
    myStepper.step(-stepsPerRevolution);
    delay(1000);
    lcd.clear();
    Serial.println(i);
  }
}

void sinewave(){
  gen.Begin();
  gen.ApplySignal(SINE_WAVE, REG0, Frequency);
  gen.EnableOutput(true); 
  Blynk.virtualWrite(V13, 39);  
}

void trianglewave(){
  gen.Begin();
  gen.ApplySignal(TRIANGLE_WAVE, REG0, Frequency);
  gen.EnableOutput(true); 
  Blynk.virtualWrite(V13, 39);   
}

void squarewave(){
  gen.Begin();
  gen.ApplySignal(SQUARE_WAVE, REG0, Frequency);
  gen.EnableOutput(true); 
  Blynk.virtualWrite(V13, 39); 
}

void ultrasonic()
{
  //ultrasonic 1
  // Clears the trigPin condition
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  distance1 = duration1 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Blynk.virtualWrite(V14, distance1);
  
  delay(1000);

  if (distance1 < 10)
  {
    Serial.println("Obstacle Detected by 1");
    //led1.on();
    
  }
  else{
    //led1.off();
  }
}
