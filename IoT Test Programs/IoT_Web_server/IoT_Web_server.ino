#include <WiFi.h>
#include <Servo.h>
#include <Stepper.h>
#include <Wire.h> 
#include <LiquidCrystal.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

//const int buzzer = 5;

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       39 // ESP32 pin GIOP39 (ADC0) connected to LM35

#define LIGHT_SENSOR_PIN 36 // ESP32 pin GIOP36 (ADC0)

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal lcd(22, 21, 1, 3, 17, 16);

const int stepsPerRevolution = 1048;  // change this to fit the number of steps per revolution

// Variable to store the HTTP request
String header;

Servo servoMotor;

// Auxiliar variables to store the current output state
String buzz5State = "off";
String servo18State = "off";
String dcmotor14State = "off";
String stepper33State = "off";
String temp39State = "off";
String ldr36State = "off";



// Assign output variables to GPIO pins
const int buzzer5out = 15;
const int servo18out = 12;
const int dcmotor14out = 14;
//const int output33 = 33;
const int temp39out = 34;
const int ldr36out = 36;

// L293D Motor Driver Pins
#define stepperIN1 13
#define stepperIN2 26
#define stepperIN3 27
#define stepperIN4 33

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, stepperIN1, stepperIN3, stepperIN2, stepperIN4);

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

  servoMotor.attach(servo18out);

  // set the speed at 5 rpm
  myStepper.setSpeed(5);
  // initialize the serial port
  Serial.begin(115200);

  //pinMode(buzzer, OUTPUT);
  
  // Initialize the output variables as outputs
  pinMode(buzzer5out, OUTPUT);
  pinMode(servo18out, OUTPUT);
  pinMode(dcmotor14out, OUTPUT);
  //pinMode(output33, OUTPUT);
  pinMode(temp39out, OUTPUT);
  pinMode(ldr36out, OUTPUT);

  // Set outputs to LOW
  digitalWrite(buzzer5out, LOW);
  digitalWrite(servo18out, LOW);
  digitalWrite(dcmotor14out, LOW);
  //digitalWrite(output33, LOW);
  digitalWrite(temp39out, LOW);
  digitalWrite(ldr36out, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void loop(){
  webserver();
  temperature();
}

void webserver(){
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 32;
  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("Buzzer on");
              buzz5State = "on";
              lcd.print ("Buzzer On");
              buzz();
              //digitalWrite(buzzer5out, HIGH);
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("Buzzer off");
              buzz5State = "off";
              lcd.clear();
            } else if (header.indexOf("GET /18/on") >= 0) {
              Serial.println("Servo Motor on");
              servo18State = "on";
              lcd.print ("Servo Motor On");
              servomotor();
            } else if (header.indexOf("GET /18/off") >= 0) {
              Serial.println("Servo Motor off");
              servo18State = "off";
              lcd.clear();
            } else if (header.indexOf("GET /14/on") >= 0) {
              Serial.println("DC Motor on");
              dcmotor14State = "on";
              lcd.print ("DC Motor On");
              digitalWrite(dcmotor14out, HIGH);
            } else if (header.indexOf("GET /14/off") >= 0) {
              Serial.println("DC Motor off");
              dcmotor14State = "off";
              digitalWrite(dcmotor14out, LOW);
              lcd.clear();
            } else if (header.indexOf("GET /33/on") >= 0) {
              Serial.println("Stepper Motor on");
              stepper33State = "on";
              lcd.print ("Stepper Motor On");
              steppermotor();
            } else if (header.indexOf("GET /33/off") >= 0) {
              Serial.println("Stepper Motor off");
              stepper33State = "off";
              lcd.clear();
            } else if (header.indexOf("GET /39/on") >= 0) {
              Serial.println("Temperature Sensor on");
              temp39State = "on";
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
            } else if (header.indexOf("GET /39/off") >= 0) {
              Serial.println("Temperature Sensor off");
              temp39State = "off";
              lcd.clear();
            } else if (header.indexOf("GET /36/on") >= 0) {
              Serial.println("LDR on");
              ldr36State = "on";
              lcd.print ("LDR Value :");
              // reads the input on analog pin (value between 0 and 4095)
              int analogValue = analogRead(LIGHT_SENSOR_PIN);
              Serial.print("Analog Value = ");
              //lcd.print("Analog Value :");
              Serial.print(analogValue);   // the raw analog reading
              // We'll have a few threshholds, qualitatively determined
              lcd.print(analogValue);
              lcd.setCursor(0,1);
              if (analogValue < 40) {
              Serial.println(" => Dark");
              lcd.print("=> Dark");
              } else if (analogValue < 800) {
              Serial.println(" => Dim");
              lcd.print("=> Dim");
              } else if (analogValue < 2000) {
              Serial.println("=> Light");
              lcd.print("=> Light");
              } else if (analogValue < 3200) {
              Serial.println(" => Bright");
              lcd.print("=> Bright");
              } else {
              Serial.println(" => Very bright");
              lcd.print("=> Very bright");
              }
            } else if (header.indexOf("GET /36/off") >= 0) {
              Serial.println("LDR off");
              ldr36State = "off";
              lcd.clear();
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 5  
            client.println("<p>Buzzer " + buzz5State + "</p>");
            // If the buzzzer is off, it displays the ON button       
            if (buzz5State=="off") {
              client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for GPIO 18
            client.println("<p>Servo Motor " + servo18State + "</p>");
            // If the servo motor is off, it displays the ON button       
            if (servo18State=="off") {
              client.println("<p><a href=\"/18/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/18/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for GPIO 14
            client.println("<p>DC Motor State " + dcmotor14State + "</p>");
            // If the DC Motor is off, it displays the ON button       
            if (dcmotor14State=="off") {
              client.println("<p><a href=\"/14/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/14/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
               
            // Display current state, and ON/OFF buttons for GPIO 33  
            client.println("<p>Stepper Motor " + stepper33State + "</p>");
            // If the Stepper Motor is off, it displays the ON button       
            if (stepper33State=="off") {
              client.println("<p><a href=\"/33/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/33/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for GPIO 39  
            client.println("<p>Temperature Sensor " + temp39State + "</p>");
            // If the Temperature Sensor is off, it displays the ON button       
            if (temp39State=="off") {
              client.println("<p><a href=\"/39/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/39/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for GPIO 36 
            client.println("<p>LDR Sensor " + ldr36State + "</p>");
            // If the LDR Sensor is off, it displays the ON button       
            if (ldr36State=="off") {
              client.println("<p><a href=\"/36/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/36/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void buzz(){
  for (int i=5; i>0; i--){
  tone(buzzer5out, 1000);
  delay(1000);
  noTone(buzzer5out);
  delay(1000);
  Serial.println(i);
  }  
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

void steppermotor(){
  for (int i=3; i>0; i--){
    // step one revolution in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    delay(1000);

    // step one revolution in the other direction:
    Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(1000);
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
}
