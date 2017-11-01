/*
SpyCamera.h - Library for using the  Adafruit "Mini Spy Camera with Trigger for Photo or Video".
Created by Louis Habberfield-Short, Spetember 9, 2017.
Released into the public domain.
*/

#include "Arduino.h"
#include "SpyCamera.h"

SpyCamera::SpyCamera(int pin)
{
	pinMode(pin, OUTPUT);
	_pin = pin;
}

void SpyCamera::takePicture()
{
	digitalWrite(_pin, LOW);
	delay(100);
	digitalWrite(_pin, HIGH);
}

void SpyCamera::toggleVideo()
{
	digitalWrite(_pin, LOW);
	delay(600);
	digitalWrite(_pin, HIGH);
}

void SpyCamera::takeVideo(int duration)
{
	toggleVideo();
	delay(duration);
	toggleVideo();
}