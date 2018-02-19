#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// wifi settings
const char* ssid0 = "NETGEAR68";
const char* password0 = "jollymint969";

unsigned int localPort =  1808 ;      // local port to listen on
WiFiUDP Udp;

int value,r,g,b;
int leftint=0;
int rightint=0;
int leftdata=0;
int rightdata=0;
char packetBuffer[255]; //buffer to hold incoming packet
char command_init[1];
char  ReplyBuffer[] = "141414141";       // a string to send back
char *pt;
char *sec;
float acc=0;
float til=0;
float ltilt=0;
float rtilt=0;
int motleft=0;
int motright=0;
int comman=0;
int xdata=152;
#define IN1  14
#define IN2  12
#define IN3  13
#define IN4  15
int Steps = 0;
boolean Direction = true;// gre
unsigned long last_time;
unsigned long currentMillis ;
int steps_left=4095;
long time1;


void setup()
{ 
  setupSerial();
  pinsetup();
  setupudp();
  motorstop();
  setupacel();
  
}

void loop() {
    aceldata();
            if (Serial.available() > 0) {
                stepscall();
        }


    //readdata(leftdata,rightdata);
}


void readdata(int leftdata,int rightdata)
{
    int packetSize = Udp.parsePacket();
    if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      stepscall();
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
    
//    analogWrite(motor1Enable, motleft);
//    analogWrite(motor2Enable, motright);    
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
  pinMode(IN1, OUTPUT); 
pinMode(IN2, OUTPUT); 
pinMode(IN3, OUTPUT); 
pinMode(IN4, OUTPUT); 
}

void setupudp()
{
//  if(digitalRead(trigPin)==HIGH)
//  {
//  WiFi.begin(ssid0, password0);// Connect to WiFi 
//  WiFi.config(IPAddress(192, 168, 1, 64),
//            IPAddress(192, 168, 1, 1),
//              IPAddress(255, 255, 255, 0));
//  }else
  {
  WiFi.begin(ssid0, password0);// Connect to WiFi 
  WiFi.config(IPAddress(192, 168, 1, 64),
  IPAddress(192, 168, 1, 1),
  IPAddress(255, 255, 255, 0));
  }
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


void motorforward()
{
    //motors forward
//  digitalWrite(motor1Forward, HIGH);
//  digitalWrite(motor1Back, LOW);
//  digitalWrite(motor2Forward, HIGH);
//  digitalWrite(motor2Back, LOW);
}


void motorreverse()
{
    //motors forward
//  digitalWrite(motor1Forward, LOW);
//  digitalWrite(motor1Back, HIGH);
//  digitalWrite(motor2Forward, LOW);
//  digitalWrite(motor2Back, HIGH);
}

void motorstop()
{
    //motors forward
//  digitalWrite(motor1Forward, LOW);
//  digitalWrite(motor1Back, LOW);
//  digitalWrite(motor2Forward, LOW);
//  digitalWrite(motor2Back, LOW);
}



void aceldata(){
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */
  //Serial.print("X Raw: "); Serial.print(accel.raw.x); Serial.print("  ");
  //Serial.print("Y Raw: "); Serial.print(accel.raw.y); Serial.print("  ");
  //Serial.print("Z Raw: "); Serial.print(accel.raw.z); Serial.println("");
  xdata=152;
  /* Delay before the next sample */
  senddata(xdata);
  delay(500);
}


void setupacel(){
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    yield();
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void stepper(int xw){
  for (int x=0;x<xw;x++){
switch(Steps){
   case 0:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
   break; 
   case 1:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, HIGH);
   break; 
   case 2:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
   break; 
   case 3:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, HIGH);
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
   break; 
   case 4:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, HIGH);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
   break; 
   case 5:
     digitalWrite(IN1, HIGH); 
     digitalWrite(IN2, HIGH);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
   break; 
     case 6:
     digitalWrite(IN1, HIGH); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
   break; 
   case 7:
     digitalWrite(IN1, HIGH); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
   break; 
   default:
     digitalWrite(IN1, LOW); 
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
   break; 
}
SetDirection();
}
}

 
void SetDirection(){
if(Direction==1){ Steps++;}
if(Direction==0){ Steps--; }
if(Steps>7){Steps=0;}
if(Steps<0){Steps=7; }
}


void stepscall()
{
   while(steps_left>0){
  currentMillis = micros();
  if(currentMillis-last_time>=1000){
  stepper(1); 
  time1=time1+micros()-last_time;
  last_time=micros();
  steps_left--;
  }
  }
   Serial.println(time1);
  Serial.println("Wait...!");
  delay(2000);
  Direction=!Direction;
  //steps_left=4095;
}


