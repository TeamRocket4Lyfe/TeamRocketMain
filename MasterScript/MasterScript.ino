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

// Define constants
#define cardSelect 10
#define NUM_DATA 15

// Set the pins used
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
char filename[15];
float timeStamp;

// Initialise sensor working variables
bool accelWorking;
bool gyroWorking;
bool magWorking;
bool bmpWorking;
bool SDWorking;

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
  float temp;
  float pressure;
} bmpReadings;

void setup() {

  // Check sensors and SD card are functional
  accelWorking = accel.begin();
  gyroWorking = gyro.begin();
  magWorking = mag.begin();
  bmpWorking = bmp.begin();
  SDWorking = SD.begin(cardSelect);

  // Find unique name for data file
  strcpy(filename, "data00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // Create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // Create and open data file for writing
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    // do something
  }

  // Create headings in data file
  logfile.print("TimeStamp(ms),Temperature(*C),Pressure(Pa),AccelX(m/s^2),AccelY(m/s^2),");
  logfile.println("AccelZ(m/s^2),MagX(uT),MagY(uT),MagZ(uT),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),OriPitch,OriRoll,OriHeading"); 

  // Close data file
  logfile.close();
}

void loop() {

  // Get a timestamp
  timeStamp = millis(); 

  // Take temperature and pressure readings if sensor is working
  if (bmpWorking) {
    bmpReadings.temp = bmp.readTemperature();
    bmpReadings.pressure = (float) bmp.readPressure();
  }

  // Allow accel event to be accessed later in the script
  sensors_event_t accel_event;
  
  // Take acceleration readings if sensor is working
  if (accelWorking) {
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
  if (magWorking) {
    // Create event
    mag.getEvent(&mag_event);

    // Take readings in 3 dimensions
    magReadings.x = mag_event.magnetic.x;
    magReadings.y = mag_event.magnetic.y;
    magReadings.z = mag_event.magnetic.z;
  }

  // Take gyroscopic readings if sensor is working
  if (gyroWorking) {
    // Create event
    sensors_event_t gyro_event;
    gyro.getEvent(&gyro_event);

    // Take readings in 3 dimensions
    gyroReadings.x = gyro_event.gyro.x;
    gyroReadings.y = gyro_event.gyro.y;
    gyroReadings.z = gyro_event.gyro.z;
  }

  // Verify if orientation can be calculated
  if (accelWorking && magWorking) {
    sensors_vec_t orientation;

    // Calculate orientation
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
      oriReadings.oriRoll = orientation.roll;
      oriReadings.oriPitch = orientation.pitch;
      oriReadings.oriHeading = orientation.heading;
    }
  }

  // Print collected data to file
  logfile = SD.open(filename, FILE_WRITE);
  
  float* dataItem[15] = {&timeStamp, &bmpReadings.temp, &bmpReadings.pressure, &accelReadings.x, &accelReadings.y, &accelReadings.z, &magReadings.x, &magReadings.y, 
    &magReadings.z, &gyroReadings.x, &gyroReadings.y, &gyroReadings.z, &oriReadings.oriRoll, &oriReadings.oriPitch, &oriReadings.oriHeading};

  for (int i = 0; i < NUM_DATA; i++) {
    logfile.print(*dataItem[i]);
    logfile.print(",");
  }
  
  logfile.println("");


  
  logfile.close(); // Save data

  // Delay 0.1 secs
  delay(100);
}
