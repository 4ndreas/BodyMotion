#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <OSCBundle.h>
#include <OSCTiming.h>


#define PIN_RESET 255  //
#define DC_JUMPER 0  // I2C Addres: 0 - 0x3C, 1 - 0x3D

// WiFi
const char* ssid = "Luftnetz_2GHz";
const char* password = "achtung!warnschuss!";

WiFiUDP Udp;

IPAddress outIp(192, 168, 2, 155);
const unsigned int outPort = 57111;
const unsigned int localUdpPort = 57112;


//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////

MicroOLED oled(PIN_RESET, DC_JUMPER);  // I2C Example



/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)
Adafruit_BNO055 bno = Adafruit_BNO055();


void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

    oled.begin();     // Initialize the OLED
  oled.clear(PAGE); // Clear the display's internal memory
  oled.clear(ALL);  // Clear the library's display buffer
  oled.display();   // Display what's in the buffer (splashscreen)
  
  oled.clear(ALL);


  Serial.printf("Connecting to %s ", ssid);
  oled.setCursor(0,0); 
  oled.printf("Connecting to %s ", ssid);
  oled.display(); // display the memory buffer drawn
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  
  setacel(0,0,0);
  oled.display(); // display the memory buffer drawn

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  imu::Quaternion quat = bno.getQuat();
  
  // Quaternion data
  
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  oled.clear(PAGE);  // Clear the buffer

  setacel(euler.x(),euler.y(),euler.z());
  oled.display(); // display the memory buffer drawn

    OSCBundle bndl;

    // euler.x = yaw (heading)
    // euler.y = ptich
    // euler.z = roll

    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
    bndl.add("/IMU/0").add((float)quat.w())
                      .add((float)quat.y())
                      .add((float)quat.x())
                      .add((float)quat.z())
                      .add((float)euler.x())  
                      .add((float)euler.y())
                      .add((float)euler.z());
                      
    Udp.beginPacket(outIp, outPort);
        bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


void setacel(int x, int y , int z)
{ 
  oled.setFontType(0); // set font type 0, please see declaration in SFE_MicroOLED.cpp
  oled.setCursor(0,0); 
  oled.print(x);
  oled.setCursor(0,10); 
  oled.print(y);
  oled.setCursor(0,20); 
  oled.print(z);
}
