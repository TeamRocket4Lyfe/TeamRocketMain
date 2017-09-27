/*
SpyCamera.h - Library for using the Adafruit "Mini Spy Camera with Trigger for Photo or Video".
Created by Louis Habberfield-Short, Spetember 9, 2017.
Released into the public domain.
*/

#ifndef SpyCamera_h
#define SpyCamera_h

#include "Arduino.h"

class SpyCamera
{
public:
	SpyCamera(int pin);
	void takePicture();
	void toggleVideo();
	void takeVideo(int duration);
private:
	int _pin;
};

#endif