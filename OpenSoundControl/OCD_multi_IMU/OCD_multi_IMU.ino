#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <OSCBundle.h>
#include <OSCTiming.h>


// MPU1 
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// MPU2
MPU6050 mpu2(0x69);
uint16_t packetSize2;
uint16_t fifoCount2;
uint8_t fifoBuffer2[64];
Quaternion q2;
VectorFloat gravity2;
float ypr2[3];



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

//MicroOLED oled(PIN_RESET, DC_JUMPER);  // I2C Example



/* This driver reads raw data from the BNO055

   Connections
   ===========
   WeMos ESP 8266
   Connect SCL to D1
   Connect SDA to D2
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define OSC_SAMPLERATE_DELAY_MS (20)


#define BNO055_SAMPLERATE_DELAY_MS (20)
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> euler;
imu::Quaternion quat;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  Wire.setClock(400000);
  
//  oled.begin();     // Initialize the OLED
//  oled.clear(PAGE); // Clear the display's internal memory
//  oled.clear(ALL);  // Clear the library's display buffer
//  oled.display();   // Display what's in the buffer (splashscreen)
  
  //oled.clear(ALL);

  Serial.printf("Connecting to %s ", ssid);
//  oled.setCursor(0,0); 
//  oled.printf("Connecting to %s ", ssid);
//  oled.display(); // display the memory buffer drawn
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);


  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

    mpu.initialize();
    mpu.dmpInitialize();
//    mpu.setXAccelOffset(-1343);
//    mpu.setYAccelOffset(-1155);
//    mpu.setZAccelOffset(1033);
//    mpu.setXGyroOffset(19);
//    mpu.setYGyroOffset(-27);
//    mpu.setZGyroOffset(16);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    mpu2.initialize();
    mpu2.dmpInitialize();
//    mpu2.setXAccelOffset(-1343);
//    mpu2.setYAccelOffset(-1155);
//    mpu2.setZAccelOffset(1033);
//    mpu2.setXGyroOffset(19);
//    mpu2.setYGyroOffset(-27);
//    mpu2.setZGyroOffset(16);
    mpu2.setDMPEnabled(true);
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
    fifoCount2 = mpu2.getFIFOCount();

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
  Wire.setClock(800000);
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  BNO055Worker();
  MPU6050Worker();
  MPU6050Worker2();
  OSCWorker();
}


void OSCWorker()
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= OSC_SAMPLERATE_DELAY_MS) 
  {
      previousMillis = currentMillis; 
      
      OSCBundle bndl;
      
      bndl.add("/IMU/1").add((float)q.w)
                        .add((float)q.y)
                        .add((float)q.x)
                        .add((float)q.z)
                        .add((float)ypr[0])  
                        .add((float)ypr[1])
                        .add((float)ypr[2]);
                        
      Udp.beginPacket(outIp, outPort);
      bndl.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket(); // mark the end of the OSC Packet
      bndl.empty(); // empty the bundle to free room for a new one

      bndl.add("/IMU/2").add((float)q2.w)
                        .add((float)q2.y)
                        .add((float)q2.x)
                        .add((float)q2.z)
                        .add((float)ypr2[0])  
                        .add((float)ypr2[1])
                        .add((float)ypr2[2]);
                        
      Udp.beginPacket(outIp, outPort);
      bndl.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket(); // mark the end of the OSC Packet
      bndl.empty(); // empty the bundle to free room for a new one


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
  }
}

void MPU6050Worker()
{
    fifoCount = mpu.getFIFOCount();
    if(fifoCount > packetSize-1)
    {
      if (fifoCount == 1024) {
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
      }
      else{
        if (fifoCount % packetSize != 0) {  
            mpu.resetFIFO(); 
      }
          else{
              while (fifoCount >= packetSize) {          
                  mpu.getFIFOBytes(fifoBuffer,packetSize);
                  fifoCount -= packetSize;     
              }    
              mpu.dmpGetQuaternion(&q,fifoBuffer);
              mpu.dmpGetGravity(&gravity,&q);
              mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
      }
      }
  }
}

void MPU6050Worker2()
{
    fifoCount2 = mpu2.getFIFOCount();
    if(fifoCount2 > packetSize2-1)
    {
      if (fifoCount2 == 1024) {
          mpu2.resetFIFO();
          Serial.println(F("FIFO MPU2 overflow!"));
      }
      else{
        if (fifoCount2 % packetSize2 != 0) {  
            mpu2.resetFIFO(); 
      }
          else{
              while (fifoCount2 >= packetSize2) {          
                  mpu2.getFIFOBytes(fifoBuffer2,packetSize2);
                  fifoCount2 -= packetSize2;     
              }    
              mpu2.dmpGetQuaternion(&q2,fifoBuffer2);
              mpu2.dmpGetGravity(&gravity2,&q2);
              mpu2.dmpGetYawPitchRoll(ypr2,&q2,&gravity2);
      }
      }
  }
}


void BNO055Worker()
{  
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= BNO055_SAMPLERATE_DELAY_MS) 
  {
      previousMillis = currentMillis;
      
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      quat = bno.getQuat();
      
    //  oled.clear(PAGE);  // Clear the buffer
    //  setacel(euler.x(),euler.y(),euler.z());
    //  oled.display(); // display the memory buffer drawn
    
      //OSCBundle bndl;
    
        // euler.x = yaw (heading)
        // euler.y = ptich
        // euler.z = roll
    
        //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
    }
}
