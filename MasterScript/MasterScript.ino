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
#define NUM_DATA 19

// Set the pins used
#define GPSSerial Serial1
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

// Assign a unique ID to the sensors

Adafruit_GPS GPS(&GPSSerial);
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP280 bmp;

// Create camera object on pin 10 (move to 9?)
SpyCamera camera(10); 

// Prepare SD card file
File logfile;
char filename[15];

// Prepare WiFi webserver. IP address will be 192.168.1.1
char ssid[] = "wifi101-network"; // created AP name
WiFiServer server(80);
int status = WL_IDLE_STATUS;
char callback[23] = "arduinoWiFiComCallback"; // Callback for JSONP transmisison

// Start timer
float timeStamp = millis();

// Initialise sensor working variables
bool accelWorking;
bool gyroWorking;
bool magWorking;
bool bmpWorking;
bool SDWorking;

struct gps_s {
  float gpsLat;
  float gpsLong;
  float gpsAlt;
  float gpsVel;
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
  float temp;
  float pressure;
} bmpReadings;

// Iniialise mission events
bool deployed = true; // true for testing of transmission
bool altOne = false;
bool altTwo = false;
bool altThree = false;
bool landed = false;

void setup() {

  // Initialise GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Recommended settings
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antennae status
  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version
  gpsReadings.gpsLat = 0; // Initialise data
  gpsReadings.gpsLong = 0;
  
  // Check sensors and SD card are functional
  accelWorking = accel.begin();
  gyroWorking = gyro.begin();
  magWorking = mag.begin();
  bmpWorking = bmp.begin();
  SDWorking = SD.begin(cardSelect);

  // Set up WiFi 
  WiFi.setPins(8,7,4,2); //Configure pins
  if (WiFi.status() == WL_NO_SHIELD) {
    // don't continue
    while (true);
  }  
  status = WiFi.beginAP(ssid); // Create open network
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }  
  delay(10000); // wait 10 seconds for connection (reduce?)
  server.begin(); // start the web server

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
  logfile.print("AccelZ(m/s^2),MagX(uT),MagY(uT),MagZ(uT),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s),OriPitch,OriRoll,OriHeading,");
  logfile.println("Latitude,Longitude,Altitude,Velocity"); 

  // Close data file
  logfile.close();
  
}

void loop() {

  // Check for new GPS sentence
  GPS.read();
  if (GPS.newNMEAreceived()) {
    // Serial.println("New NMEA received");
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
    
    float* dataItem[19] = {&timeStamp, &bmpReadings.temp, &bmpReadings.pressure, &accelReadings.x, &accelReadings.y, &accelReadings.z, 
    &magReadings.x, &magReadings.y, &magReadings.z, &gyroReadings.x, &gyroReadings.y, &gyroReadings.z, &oriReadings.oriRoll, 
    &oriReadings.oriPitch, &oriReadings.oriHeading, &gpsReadings.gpsLat, &gpsReadings.gpsLong, &gpsReadings.gpsAlt, &gpsReadings.gpsVel};
    
    for (int i = 0; i < NUM_DATA; i++) {
      logfile.print(*dataItem[i], 4); // Print values to 4 dp
      logfile.print(",");
    }
    
    logfile.println("");
    
    logfile.close(); // Save data

    if (deployed) {
      WiFiClient client = server.available();   // listen for incoming clients

      if (client) {                             // if you get a client,
        String currentLine = "";                // make a String to hold incoming data from the client
        while (client.connected()) {            // loop while the client's connected
          if (client.available()) {             // if there's bytes to read from the client,
            char c = client.read();             // read a byte, then
            if (c == '\n') {                    // if the byte is a newline character
    
              // if the current line is blank, you got two newline characters in a row.
              // that's the end of the client HTTP request, so send a response:
              if (currentLine.length() == 0) {
                // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                // and a content-type so the client knows what's coming, then a blank line:
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:application/json");
                client.println();
    
                // the content of the HTTP response follows the header:
                client.print(callback);
                client.print("('{");
                for (int i = 0; i < NUM_DATA; i++) {
                  client.print("\"A");
                  client.print(i);
                  client.print("\": ");
                  client.print(*dataItem[i], 4); // Print values to 4 dp
                  if (i != (NUM_DATA -1)) client.print(",");                  
                }
                
                // The HTTP response ends with another blank line:
                client.println("}')");
                // break out of the while loop:
                break;
              }
              else {      // if you got a newline, then clear currentLine:
                currentLine = "";
              }
            }
            else if (c != '\r') {    // if you got anything else but a carriage return character,
              currentLine += c;      // add it to the end of the currentLine
            }
          }
        }
    
        delay(1);
        // close the connection:
        client.stop();
      }
    }
  }

//  // Check mission events
//
//  if (!deployed // && deployment condition) {
//    deployed = true;
//    // Ready to start transmission
//  }
//  
  if (deployed && !altOne /* && altOne reached */) {
    // Triggered from barometer data, take two pictures
    altOne = true;
    camera.takePicture(); camera.takePicture();
  }
//
//  if (deployed && !altTwo // && altTwo reached) {
//    // Triggered from GPS data, take two pictures
//    altTwo = true;
//    camera.takePicture(); camera.takePicture();
//  }
//
//  if (deployed && !altThree // && altThree reached) {
//    altThree = true;
//    // Start video landing
//    camera.toggleVideo();
//  }
//
//   if (deployed && !landed // && landed reached) {
//    landed = true;
//    camera.toggleVideo();
//  }
}
