// Inbuilt Adafruit libraries
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

// Sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>

// Set the pins used
#define cardSelect 10
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

// Assign a unique ID to the sensors
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP280 bmp;

File logfile;

void setup() {

  // Check sensors and SD card are functional
  bool accelWorking = accel.begin();
  bool gyroWorking = gyro.begin();
  bool magWorking = mag.begin();
  bool bmpWorking = bmp.begin();
  bool SDWorking = SD.begin(cardSelect);

  // Find unique name for data file
  char filename[15];
  strcpy(filename, "DATA00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // Create and open data file for writing
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    // do something
  }
}

void loop() {
  
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  // Read the accelerometer and magnetometer
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  logfile.print("Temperature = "); 
  logfile.print(bmp.readTemperature()); 
  logfile.print(" *C, ");
  
  logfile.print(F("Pressure = ")); 
  logfile.print(bmp.readPressure()); 
  logfile.print(" Pa, ");

  // Pressure should be adjusted
  logfile.print(F("Approx altitude = ")); 
  logfile.print(bmp.readAltitude(1013.25)); 
  logfile.println(" m");
  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    // 'orientation' should have valid .roll and .pitch fields
    logfile.print(F("Orientation: ")); 
    logfile.print(orientation.roll); 
    logfile.print(F(", ")); 
    logfile.print(orientation.pitch); 
    logfile.print(F(", ")); 
    logfile.print(orientation.heading); 
    logfile.println(F(", "));
  }
  
  delay(100);

  if (millis() >= 20000){
    logfile.close();
  }
  while (millis() >= 20000){
  }
}
