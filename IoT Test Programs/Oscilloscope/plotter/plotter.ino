

// Import required libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPAsyncWebServer.h>
  #include <SPIFFS.h>
#else
  #include <Arduino.h>
  //#include <ESP8266WiFi.h>
  #include <Hash.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <FS.h>
#endif
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <AD9833.h>     // Include the library

#define FNC_PIN 5       // Can be any digital IO pin
#define Frequency 1000      // Frequency = 1Hz

const int channelone = 39; // I2C

//const int channeltwo = 35; // I2C

//const int channelthree = 32; // I2C

//const int channelfour = 34; // I2C

//--------------- Create an AD9833 object ----------------
// Note, SCK and MOSI must be connected to CLK and DAT pins on the AD9833 for SPI
AD9833 gen(FNC_PIN);       // Defaults to 25MHz internal reference frequency

// Replace with your network credentials
const char* ssid = "Guest-310";
const char* password = "SPIT@30092k22etrxap2";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);







String first() {
  gen.Begin();
  
  //gen.ApplySignal(SINE_WAVE, REG0, Frequency);
  //gen.ApplySignal(TRIANGLE_WAVE,REG0,Frequency);
  gen.ApplySignal(SQUARE_WAVE,REG0,Frequency);
  //gen.ApplySignal(HALF_SQUARE_WAVE,REG0,Frequency);

  gen.EnableOutput(true);   // Turn ON the output - it defaults to OFF
  // There should be a 1 Hz square wave on the output of the AD9833

  float t = analogRead(channelone);
  


  return String(t);
}

/*String second() {

  float u = analogRead(channeltwo);
  
  Serial.println(u);

  return String(u);
}

String third() {

  float v = analogRead(channelthree);
  


  return String(v);
}
*/









void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);


  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());






  

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });


  
  server.on("/firstchannel", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", first().c_str());
  });
  /*server.on("/secondchannel", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", second().c_str());
  });
  server.on("/thirdchannel", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", third().c_str());
  });*/

  // Start server
  server.begin();
}
 
void loop(){
  //Serial.println(analogRead(39));

  //delay(10);
}
