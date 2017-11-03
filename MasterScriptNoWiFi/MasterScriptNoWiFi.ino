// Inbuilt Adafruit libraries
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WiFi101.h>

// Sensor libraries
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>

// Custom libraries
#include <SpyCamera.h>

// Define constants
#define cardSelect 10
#define NUM_DATA 21
#define GPSSerial Serial1
#define GPS_BEGIN 9600

// Set the pins used
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
#define VBATPIN A7

// Assign a unique ID to the sensors
Adafruit_GPS GPS(&GPSSerial);
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP280 bmp;
SpyCamera camera(6); // On pin 6

// Prepare SD card file
File logfile;
char filename[15];

// Start timer
float timeStamp = millis();

// Initialise mission events
bool gpsAltInitialised = false;
bool deployed = false; // make true for testing purposes
bool altOne = false;
bool altTwo = false;
bool altThree = false;
bool landed = false;

// Create sensor structures
struct gps_s {
  float gpsLat;
  float gpsLong;
  float gpsAlt;
  float gpsVel;
  float gpsGroundAlt;
} gpsReadings;

struct accel_s {
  float x;
  float y;
  float z;
} accelReadings;

struct mag_s {
  float x;
  float y;
  float z;
} magReadings;

struct gyro_s {
  float x;
  float y;
  float z;
} gyroReadings;

struct ori_s {
  float oriRoll;
  float oriPitch;
  float oriHeading;
} oriReadings;

struct bmp_s {
  float groundPressure;
  float temp;
  float pressure;
  float altitude;
} bmpReadings;

float batteryVoltage;

void setup() {
  
  initialiseGPS();

  getDataFileName();
  
  if (SD.begin(cardSelect)) {
    printDataHeadingsInLogfile();
  }

  if (bmp.begin()) {
    recordGroundPressure();
  }
}

/*
 * Sets up the GPS to take latitudinal and longitudinal data
 */
void initialiseGPS() {
  GPS.begin(GPS_BEGIN);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Recommended settings
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antennae status
  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version
  gpsReadings.gpsLat = 0; // Initialise data
  gpsReadings.gpsLong = 0;
}

/**
 * Finds a unique filename to store the sensor data in and stores this name in the 
 * filename variable
 */
 void getDataFileName() {
  strcpy(filename, "data00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // Create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
 }

 /**
  * Prints a heading into the logfile to indicate which column corresponds to which sensor reading
  */
void printDataHeadingsInLogfile() {
  
  // Create and open data file for writing
  logfile = SD.open(filename, FILE_WRITE);

  // Create headings in data file
  logfile.print("TimeStamp(ms),Temperature(*C),Pressure(Pa),Altitude(m),AccelX(m/s^2),AccelY(m/s^2),");
  logfile.print("AccelZ(m/s^2),MagX(uT),MagY(uT),MagZ(uT),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),OriPitch,OriRoll,OriHeading,");
  logfile.println("Latitude,Longitude,Altitude,Velocity,Voltage");

  // Close data file
  logfile.close();
}

void recordGroundPressure() {
  bmpReadings.groundPressure = bmp.readPressure()/100; // Convert to hectopascals/millibars
}

void loop() {

  // Check for new GPS sentence
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA())) 
      return; // we can fail to parse a sentence in which case we should just wait for another     
  }

  // Perform read/write once every second
  if (millis() - timeStamp > 1000) {
    timeStamp = millis(); // Set the timeStamp
    
    if (GPS.fix) {
        gpsReadings.gpsLat = GPS.latitudeDegrees;
        gpsReadings.gpsLong = GPS.longitudeDegrees;
        gpsReadings.gpsAlt = GPS.altitude;
        gpsReadings.gpsVel = GPS.speed;
      }
    
    // Take temperature and pressure readings if sensor is working
    if (bmp.begin()) {
      bmpReadings.temp = bmp.readTemperature();
      bmpReadings.pressure = (float) bmp.readPressure();
      bmpReadings.altitude = bmp.readAltitude(bmpReadings.groundPressure);
    }
  
    // Allow accel event to be accessed later in the script
    sensors_event_t accel_event;
    
    // Take acceleration readings if sensor is working
    if (accel.begin()) {
      // Create acceleration event
      accel.getEvent(&accel_event);
  
      // Take readings in 3 dimensions
      accelReadings.x = accel_event.acceleration.x;
      accelReadings.y = accel_event.acceleration.y;
      accelReadings.z = accel_event.acceleration.z;
    }
  
    // Allow magnetic event to be accessed later in the script
    sensors_event_t mag_event;
  
    // Take magnetic field strength readings if sensor is working
    if (mag.begin()) {
      // Create event
      mag.getEvent(&mag_event);
  
      // Take readings in 3 dimensions
      magReadings.x = mag_event.magnetic.x;
      magReadings.y = mag_event.magnetic.y;
      magReadings.z = mag_event.magnetic.z;
    }
  
    // Take gyroscopic readings if sensor is working
    if (gyro.begin()) {
      // Create event
      sensors_event_t gyro_event;
      gyro.getEvent(&gyro_event);
  
      // Take readings in 3 dimensions
      gyroReadings.x = gyro_event.gyro.x;
      gyroReadings.y = gyro_event.gyro.y;
      gyroReadings.z = gyro_event.gyro.z;
    }

    // Verify if orientation can be calculated
    if (accel.begin() && mag.begin()) {
      sensors_vec_t orientation;
  
      // Calculate orientation
      if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
        oriReadings.oriRoll = orientation.roll;
        oriReadings.oriPitch = orientation.pitch;
        oriReadings.oriHeading = orientation.heading;
      }
    }

    // Read battery voltage
    batteryVoltage = analogRead(VBATPIN);
    batteryVoltage *= (2*3.3/1024); // Conversion to voltage

    // Print collected data to file
    logfile = SD.open(filename, FILE_WRITE);

    if (logfile) {
        float* dataItem[21] = {&timeStamp, &bmpReadings.temp, &bmpReadings.pressure, &bmpReadings.altitude, &accelReadings.x, &accelReadings.y, &accelReadings.z, 
      &magReadings.x, &magReadings.y, &magReadings.z, &gyroReadings.x, &gyroReadings.y, &gyroReadings.z, &oriReadings.oriRoll,  &oriReadings.oriPitch, &oriReadings.oriHeading,
      &gpsReadings.gpsLat, &gpsReadings.gpsLong, &gpsReadings.gpsAlt, &gpsReadings.gpsVel, &batteryVoltage};
    
      for (int i = 0; i < NUM_DATA; i++) {
        logfile.print(*dataItem[i], 4); // Print values to 4 dp
        logfile.print(",");
      }
    
      logfile.println("");
    
      logfile.close(); // Save data
    }
    
    // Check mission events

    if (!deployed && (bmpReadings.altitude > 600)) {
      // Note - this script version does not transmit so determining exact deployment time is unnecesasary
      deployed = true;
    }

    if (deployed && !altOne && (bmpReadings.altitude < 500)) {
      // 500m descending mark triggered from barometer data, take two pictures
      altOne = true;
      camera.takePicture(); camera.takePicture();
    }
  
    if (deployed && !altTwo && (gpsReadings.gpsAlt < 300)) {
      // 300m descending mark triggered from GPS data, take two pictures
      altTwo = true;
      camera.takePicture(); camera.takePicture();
    }
  
    if (deployed && !altThree && (bmpReadings.altitude < 30)) {
      // 30m descending mark triggered from barometer data, begin video
      altThree = true;
      // Start video landing
      camera.toggleVideo();
    }
  
    if (deployed && !landed && altThree && (bmpReadings.altitude < 5)) {
      // 5m descending mark trigegred from barometer data
      landed = true;
      delay(5000); // Wait 5 seconds to esnure landed
      camera.toggleVideo(); // End video
      while (landed); // No transmission so end operation
    }
  }
}
