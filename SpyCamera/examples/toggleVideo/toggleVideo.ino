/*
  toggleVideo
  Takes a video for 10 seconds using the Adafruit spy camera. 
  Uses the toggleVideo method of the SpyCamera class. 
  Blinks the onboard LED during video recording.
  
  modified 26 Sep 2017
  by Louis Habberfield-Short
*/

#include <SpyCamera.h>
SpyCamera camera(10); // Pass in pin number

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  camera.toggleVideo();

  while (millis() < 10000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // toggle the LED state
    delay(500);
  }
  
  camera.toggleVideo();
  
  while true {
    // Finish script after video is complete
  }
}
