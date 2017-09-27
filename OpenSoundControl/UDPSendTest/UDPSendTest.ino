
/*
Make an OSC bundle and send it over UDP

OSCBundles allow OSCMessages to be grouped together
to  preserve the order and completeness of related messages.
They also allow for timetags to be carried to represent the presentation time
of the messages.
 */


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <OSCTiming.h>


const char* ssid = "Luftnetz_2GHz";
const char* password = "achtung!warnschuss!";

WiFiUDP Udp;

IPAddress outIp(192, 168, 2, 155);
const unsigned int outPort = 57111;
const unsigned int localUdpPort = 57112;


void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}



void loop(){
  //declare the bundle
    OSCBundle bndl;

    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
    bndl.add("/analog/0").add((int32_t)analogRead(0));
    bndl.add("/analog/1").add((int32_t)analogRead(1));
    bndl.add("/digital/5").add((digitalRead(5)==HIGH)?"HIGH":"LOW");

    Udp.beginPacket(outIp, outPort);
        bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

    bndl.add("/mouse/step").add((int32_t)analogRead(0)).add((int32_t)analogRead(1));
    bndl.add("/units").add("pixels");

    Udp.beginPacket(outIp, outPort);
        bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one

    delay(1000);
}
