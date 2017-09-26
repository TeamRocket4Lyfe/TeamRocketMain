/*
  takePicture
  Takes a picture every 3 seconds using the Adafruit spy camera.
  Uses the takePicture method of the SpyCamera class. 
  
  modified 26 Sep 2017
  by Louis Habberfield-Short
*/

#include <SpyCamera.h>
SpyCamera camera(10); // Pass in pin number

void setup() {
}

void loop() {
  delay(3000);
  camera.takePicture();
}
