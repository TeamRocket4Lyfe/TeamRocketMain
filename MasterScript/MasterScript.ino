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
  strcpy(filename, "DATA00.csv");
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

  // Create headings in data file
  logfile.print(F("TimeStamp(),Temperature(*C),Pressure(),AccelX(m/s^2),AccelY(m/s^2),")
  logfile.println(F("AccelZ(m/s^2),MagX(uT),MagY(uT),MagZ(uT),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),OriPitch(),OriRoll(),OriHeading()")); 
}

void loop() {

  // Initialise sensor variables
  // TIMESTAMP???
  float temp;
  int32_t pressure;
  float accelX;
  float accelY;
  float accelZ;
  float magX;
  float magY;
  float magZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float roll;
  float pitch;
  float heading;

  // Take temperature and pressure readings if sensor is working
  if (bmpWorking) {
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
  }
  
  // Take acceleration readings if sensor is working
  if (accelWorking) {
    // Create acceleration event
    sensors_event_t accel_event;
    accel.getEvent(&accel_event);

    // Take readings in 3 dimensions
    accelX = accel_event.acceleration.x;
    accelY = accel_event.acceleration.y;
    accelZ = accel_event.acceleration.z;
  }

  // Take magnetic field strength readings if sensor is working
  if (magWorking) {
    // Create event
    sensors_event_t mag_event;
    mag.getEvent(&mag_event);

    // Take readings in 3 dimensions
    magX = mag_event.magnetic.x;
    magY = mag_event.magnetic.y;
    magZ = mag_event.magnetic.z;
  }

  // Take gyroscopic readings if sensor is working
  if (gyroWorking) {
    // Create event
    sensors_event_t gyro_event;
    gyro.getEvent(&gyro_event);

    // Take readings in 3 dimensions
    gyroX = gyro_event.gyro.x;
    gyroY = gyro_event.gyro.y;
    gyroZ = gyro_event.gyro.z;
  }

  // Verify if orientation can be calculated
  if (accelWorking && magWorking) {
    sensors_vec_t orientation;

    // Calculate orientation
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
      roll = orientation.roll();
      pitch = orientation.pitch();
      heading = orientation.heading();
    }
  }

  // Print collected data to file
  logfile.println(F(timeStamp,temp,pressure,accelX,accelY,magX,magY,magZ,gyroX,gyroY,gyroZ,pitch,roll,heading))

  // Delay 0.1 secs
  delay(100);

  // After 20 secs of collecting data, close the file and do nothing
  if (millis() >= 20000){
    logfile.close();

    while (1);
  }
}
