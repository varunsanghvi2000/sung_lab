#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// wifi settings
const char* ssid0 = "NETGEAR68";
const char* password0 = "jollymint969";

unsigned int localPort =  1808 ;      // local port to listen on
WiFiUDP Udp;

char packetBuffer[255]; //buffer to hold incoming packet
char command_init[1];
char  ReplyBuffer[] = "141414141";       // a string to send back





void readdata(int leftdata,int rightdata)
{
    int packetSize = Udp.parsePacket();
    if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    //command_init[0]=packetBuffer[0];
    pt = strtok (packetBuffer,":");
    sec=pt;
    Serial.println("command ");
    Serial.println(sec);
    comman=atoi(sec);
    Serial.println("com ");
    Serial.println(comman);
    pt = strtok (NULL,":");
    Serial.println("value ");
    //Serial.println(pt);
    value=atoi(pt);
    Serial.println(value);

    if(comman==1)
    {
      Serial.println("comman ");
      Serial.println(comman);
      Serial.println("here1 ");
      Serial.println(value);
      if(value==0)
        motorstop();
      if(value==1)
        motorforward();
      if(value==2)
        motorreverse();
    }else if(comman==2)
    {
            Serial.println("comman ");
      Serial.println(comman);

      
       acc=(float)value/10;
       Serial.println("acc ");
      Serial.println(acc);
    }else if(comman==3)
    {
      Serial.println("comman ");
      Serial.println(comman);
      Serial.println("value");
      Serial.println(value);
       til=(float)value/100;
       Serial.println("value/100");
       Serial.println(til);
       if(til<0.9)
       {
       ltilt=1;
       rtilt=til+0.1; 
       Serial.println("r ltilt ");
      Serial.println(ltilt);
      Serial.println("r rtilt");
      Serial.println(rtilt);
       }else if(til>0.9)
       {
       ltilt=1-abs(til-0.9)+0.1;
       rtilt=1; 
       Serial.println("l ltilt ");
       Serial.println(ltilt);
       Serial.println("l rtilt");
       Serial.println(rtilt);
       }else{
       ltilt=1;
       rtilt=1; 
       //       Serial.println("ltilt ");
     // Serial.println(ltilt);
   //   Serial.println("rtilt");
  //    Serial.println(rtilt);
       }
    }
    
  }
  motleft=1023*acc*ltilt;
  motright=1023*acc*rtilt;
  //Serial.println("pwm_left");
  //Serial.println(motleft);
  //Serial.println("motright");
  //Serial.println(sec);
    
    analogWrite(motor1Enable, motleft);
    analogWrite(motor2Enable, motright);    
}

void senddata(int sendval)
{
    sprintf(ReplyBuffer, "%d", sendval);
    Udp.beginPacket(Udp.remoteIP(),Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
}


void pinsetup()
{
  pinMode(motor1Enable, OUTPUT);
  pinMode(motor1Forward, OUTPUT);
  pinMode(motor1Back, OUTPUT);
  pinMode(motor2Enable, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor2Back, OUTPUT);
  pinMode(trigPin, INPUT);
}

void setupudp()
{
  
  WiFi.begin(ssid0, password0);// Connect to WiFi 
  WiFi.config(IPAddress(192, 168, 1, 64),
  IPAddress(192, 168, 1, 1),
  IPAddress(255, 255, 255, 0));
  
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

}

void setupSerial()
{
    Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}


