

#include <AD9833.h>     // Include the library

#define FNC_PIN 5       // Can be any digital IO pin
#define Frequency 1000      // Frequency = 1Hz

//--------------- Create an AD9833 object ----------------
// Note, SCK and MOSI must be connected to CLK and DAT pins on the AD9833 for SPI
AD9833 gen(FNC_PIN);       // Defaults to 25MHz internal reference frequency

void setup() {
  Serial.begin(115200);
  // This MUST be the first command after declaring the AD9833 object
  gen.Begin();

  // Apply a 1000 Hz sine wave using REG0 (register set 0). There are two register sets: REG0 and REG1.
  // Each one can be programmed for:
  //   Signal type - SINE_WAVE, TRIANGLE_WAVE, SQUARE_WAVE, and HALF_SQUARE_WAVE
  //   Frequency - 0 to 12.5 MHz
  //   Phase - 0 to 360 degress (this is only useful if it is 'relative' to some other signal
  //           such as the phase difference between REG0 and REG1).
  // In ApplySignal, if Phase is not given, it defaults to 0.
  //gen.ApplySignal(SINE_WAVE, REG0, Frequency);
  //gen.ApplySignal(TRIANGLE_WAVE,REG0,Frequency);
  gen.ApplySignal(SQUARE_WAVE,REG0,Frequency);
  //gen.ApplySignal(HALF_SQUARE_WAVE,REG0,Frequency);

  gen.EnableOutput(true);   // Turn ON the output - it defaults to OFF
  // There should be a 1 Hz square wave on the output of the AD9833
}

void loop() {
  Serial.println(analogRead(39));

  delay(10);
}
