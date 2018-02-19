// library
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// variables
const char* ssid0 = "NETGEAR68";
const char* password0 = "jollymint969";
unsigned int localPort =  1808 ;      // local port to listen on
WiFiUDP Udp;
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "141414141";       // a string to send back



// prog
void setup() {
  setupSerial();
  pinsetup();
  setupudp();
}

void loop() {
  readdata();
}



// custom functions
void setupSerial()
{
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void pinsetup()
{
  //setup pins to low
}

void setupudp()
{
  WiFi.begin(ssid0, password0);// Connect to WiFi
  WiFi.config(IPAddress(192, 168, 1, 64), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  // while wifi not connected yet, print '.'
  // then after it connected, get out of the loop
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //print a new line, then print WiFi connected and the IP address
  Serial.println("");
  Serial.println("WiFi connected");
  // Print the IP address
  Serial.println(WiFi.localIP());
  Udp.begin(localPort);
  Serial.println(Udp.localPort());

}


void readdata()
{
  int packetSize = Udp.parsePacket();
  int dir = 0;
  int steps = 0;
  char *pt;
  char *sec;
  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    pt = strtok (packetBuffer, ":");
    sec = pt;
    dir = atoi(sec);
    pt = strtok (NULL, ":");
    steps = atoi(pt);
    motorrot(dir, steps);
  }
}


void senddata(int sendval)
{
  sprintf(ReplyBuffer, "%d", sendval);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Serial.print("ReplyBuffer ");
  Serial.print(ReplyBuffer);
  Udp.write(ReplyBuffer);
  Udp.endPacket();
}

void motorrot(int dir, int steps)
{
  Serial.print("dir ");
  Serial.print(dir);
  Serial.print("steps ");
  Serial.println(steps);
  senddata(1010);
}

