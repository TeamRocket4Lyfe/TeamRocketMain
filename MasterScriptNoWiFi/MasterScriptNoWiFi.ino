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

// Set the pins used
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
#define VBATPIN A7
#define CAMERA_SELECT 6
#define CARD_SELECT 10

// Define constants
#define NUM_DATA 21
#define GPSSerial Serial1
#define GPS_BEGIN 9600
#define SAMPLE_PERIOD 100  // in milliseconds
#define PRESSURE_RECORD_TIME 5000 // in milliseconds

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

float maxAltitude;

// Assign a unique ID to the sensors
Adafruit_GPS GPS(&GPSSerial);
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP280 bmp;
SpyCamera camera(CAMERA_SELECT);

// Initialise global variables
File logfile; // Prepare SD card file
char filename[15];
float batteryVoltage;
float timeStamp = millis(); // Start timer

// Initialise mission events
bool gpsAltInitialised = false;
bool deployed = false; // make true for testing purposes
bool altOne = false;
bool altTwo = false;
bool altThree = false;
bool landed = false;

void setup() {
  
  initialiseGPS();

  getDataFileName();
  
  if (SD.begin(CARD_SELECT)) {
    printDataHeadingsInLogfile();
  }

  if (bmp.begin()) {
    recordGroundPressure();
  }
}

void loop() {

  if (checkForGPSSentence()) {
    if (readyToSample()) {

      // Set the time stamp
      timeStamp = millis(); 

      // Take sensor readings
      getGPSData();
      getBMPData();
      getIMUData();
      getBatteryVoltage();

      // Print collected data to a logfile
      printDataRowToLogfile();
    
      // Take pictures at mission events
      checkForParachuteDeployment();
      checkForApogee();
      checkFor500mDescending();
      checkFor300mDescending();
      checkFor30mDescending();
      checkForLanding();
    }
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

/**
 * Records the ground pressure to use as a reference
 */
void recordGroundPressure() {
  float startTime = millis();
  while ((bmpReadings.groundPressure < 0.9*1013.25) || (bmpReadings.groundPressure > 1.1*1013.25) || (millis() < startTime + PRESSURE_RECORD_TIME)) {
    bmpReadings.groundPressure = bmp.readPressure()/100; // Convert to hectopascals/millibars
  }
}

/**
 * Checks for a new GPS sentece. Returns true if a sentence could be parsed and false otherwise.
 */
bool checkForGPSSentence() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA())) 
      return false;     
  }
  return true;
}

/**
 * Checks that more than the sample period has passed since measurements were taken.
 */
bool readyToSample() {
  if (millis() - timeStamp > SAMPLE_PERIOD) {
    return true;
  } else {
    return false;
  }
}

/**
 * Takes readings from the GPS if a GPS fix can be obtained.
 */
void getGPSData () {
  if (GPS.fix) {
    gpsReadings.gpsLat = GPS.latitudeDegrees;
    gpsReadings.gpsLong = GPS.longitudeDegrees;
    gpsReadings.gpsAlt = GPS.altitude;
    gpsReadings.gpsVel = GPS.speed;
  }
}

/**
 * Takes temperature and pressure readings from the BMP if the sensor is connected.
 */
void getBMPData() {
  if (bmp.begin()) {
    bmpReadings.temp = bmp.readTemperature();
    bmpReadings.pressure = (float) bmp.readPressure();  
    bmpReadings.altitude = bmp.readAltitude(bmpReadings.groundPressure);

     // Update maximum altitude reached
    if (bmpReadings.altitude > maxAltitude) {
      maxAltitude = bmpReadings.altitude;
    }
  }
}

/**
 * Takes acceleration, magnetic field strength and rotational motion readings from the IMU.
 */
void getIMUData() {
  
  // Create IMU events
  sensors_event_t accel_event; 
  sensors_event_t mag_event;
    
  getAccelData(accel_event);
  getMagData(mag_event);
  getGyroData();
    
  getOrientationData(accel_event, mag_event);
}

/**
 * Takes acceleration readings from the IMU if the sensor is connected.
 */
void getAccelData(sensors_event_t accel_event) {
  if (accel.begin()) {
    accel.getEvent(&accel_event);
  
    // Take readings in 3 dimensions
    accelReadings.x = accel_event.acceleration.x;
    accelReadings.y = accel_event.acceleration.y;
    accelReadings.z = accel_event.acceleration.z;
  }
}

/**
 * Takes magnetic field readings from the IMU if the sensor is connected.
 */
void getMagData(sensors_event_t mag_event) {
  if (mag.begin()) {
    // Create event
    mag.getEvent(&mag_event);
  
    // Take readings in 3 dimensions
    magReadings.x = mag_event.magnetic.x;
    magReadings.y = mag_event.magnetic.y;
    magReadings.z = mag_event.magnetic.z;
  }
}

/**
 * Takes rotational motion readings from the IMU if the sensor is connected.
 */
void getGyroData() {
  if (gyro.begin()) {
    // Create event
    sensors_event_t gyro_event;
    gyro.getEvent(&gyro_event);
  
    // Take readings in 3 dimensions
    gyroReadings.x = gyro_event.gyro.x;
    gyroReadings.y = gyro_event.gyro.y;
    gyroReadings.z = gyro_event.gyro.z;
  }
}

/**
 * Calculates orientation using accelerometer and magnetometer readings from the IMU.
 */
void getOrientationData(sensors_event_t accel_event, sensors_event_t mag_event) {
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
}

/**
 * Reads the battery voltage.
 */
void getBatteryVoltage() {
  batteryVoltage = analogRead(VBATPIN);
  batteryVoltage *= (2*3.3/1024); // Convert to voltage
}

/**
 * Prints a row of sensor readings to a logfile in csv format.
 */
void printDataRowToLogfile() {
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
}

/**
 * Check if apogee has been reached using the barometer altitude
 */
void checkForParachuteDeployment() {
  if (!deployed && ((bmpReadings.altitude > 600) || (gpsReadings.gpsAlt > 600))) {
    // Note - this script version does not transmit so determining exact deployment time is unnecesasary
    deployed = true;
  }
}

/**
 * Check if apogee has been reached by checking if PSat is less than 95% of maximum altitude and greater than 90% of maximum altitude
 */
void checkForApogee() {
  if (bmpReadings.altitude < 0.95*maxAltitude) && bmpReadings.altitude > 0.9*maxAltitude) {
    // Take one picture
    camera.takePicture();
  }
}

/**
 * Checks if the 500m descending mark has been reached using the barometer altitude.
 */
void checkFor500mDescending() {
  if (deployed && !altOne && (bmpReadings.altitude < 500)) {
    altOne = true;

    // Take two pictures
    camera.takePicture(); 
    camera.takePicture();
  }
}

/**
 * Takes two pictures if the 500m descending mark has been reached (as determined by the GPS).
 */
void checkFor300mDescending() {
  if (deployed && !altTwo && (gpsReadings.gpsAlt < 300)) {
    altTwo = true;

    // Take two pictures
    camera.takePicture(); 
    camera.takePicture();
  }
}

/**
 * Starts the video if the PSat is about to land.
 */
void checkFor30mDescending() {
  if (deployed && !altThree && (bmpReadings.altitude < 100)) {
    altThree = true;
      
    // Video landing
    camera.toggleVideo();
  }
}

/**
 * Stops the video if the PSat has landed.
 */
void checkForLanding() {
  if (!landed && altThree && (bmpReadings.altitude < 5)) {
    // 5m descending mark trigegred from barometer data
    landed = true;
    delay(5000); // Wait 5 seconds to esnure landed
    camera.toggleVideo(); // End video
    while (landed); // No transmission so end operation
  }
}

