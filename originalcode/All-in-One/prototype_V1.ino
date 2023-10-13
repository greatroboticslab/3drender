#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps; 

Adafruit_GPS GPS(&Wire);
#define GPSECHO false

uint32_t timer = millis();

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

#define Front_Left  0x80 // Address for the Front Left motor
#define Front_Right 0x81 // Address for the Front Right motor
#define Back_Left   0x82 // Address for the Back Left motor
#define Back_Right  0x83 // Address for the Back Right motor
#define Middle_Left 0x85 // Address for the Middle Left motor
#define Middle_Right 0x84 // Address for the Middle Right motor


//declaring variables
int count = 0; //count to keep track of waypoints

double clat; //current latitude
double clng; //current longitude
double chead;//current heading (direction)

double wphead;//desired direction

double headTol = 3.5; //tolerance in degree difference 
double disTol = 2.5; // tolerance of 2.5 m (advertised accuracy of gps unit)

double wplat[] = {35.8493094, 35.8498035, 35.8494888, 35.8493854}; //array to store waypoint latitudes
double wplng[] = {-86.3698679, -86.3691099, -86.3694211, -86.3697301}; //array to store waypoint longitudes
double totalPoints = 3;

//offsets for the magnometer
double xoff = 0; 
double yoff = 0;

//starts motor drivers
SoftwareSerial serial(10,11);
Roboclaw roboclaw(&serial, 10000);

void setup() 
{
  roboclaw.begin(38400);
  Serial.begin(115200);
  GPS.begin(0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("");
  Serial.println("Program Start");
  stop(5); //creates pause before robot starts moving. if not included upon the first movement command the motors won't start correctly.
  //serial.begin(9600); //starts gps
  
}



void loop() {
  // put your main code here, to run repeatedly:
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA())) 
      return; 
  }

  if (count < totalPoints)
  {
    
    if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) 
    {
      clat = GPS.latitude;
      clong = GPS.longitude;
      chead = getCurrentHeading();
      Serialprint("update 1 (lat long head): "); Serial.print(clat,6); Serial.print(clong,6); Serial.println(chead,3);
    }
  }
    wphead = TinyGPSPlus::courseTo(clat, clng, wplat[count], wplng[count]);  //calculates heading desired for waypoint
    chead = getCurrentHeading(); //uses the magnometer to get the current heading
    double distance = haversine(clat, clng, wplat[count], wplng[count]); //calculates the distance between current coords and the current waypoint
  
    while(abs(wphead - chead) > headTol)
    {
      if((wphead - chead) < 0)
      {
        turnRight(3, 70);
      }
      else if ((wphead - chead) > 0)
      {
        turnLeft(3, 70);
      }
      else
      {

      }
      chead = getCurrentHeading();
      Serial.println("update 2 (head): "); Serial.println(chead, 3);
    }

    while(distance > disTol)
    {
      moveForward(5, 50);
      char c = GPS.read();
      if (GPSECHO)
      if (c) Serial.print(c);
      if (GPS.newNMEAreceived()) 
      {
        Serial.println(GPS.lastNMEA());
        if (!GPS.parse(GPS.lastNMEA()))
        return;
      }

  // approximately every 2 seconds or so, print out the current stats
      if (millis() - timer > 2000) {
        timer = millis(); // reset the timer
        if (GPS.fix) 
        {
          clat = GPS.latitude;
          clong = GPS.longitude;
          float chead = getCurrentHeading();
          Serialprint("update 3 (lat long head): "); Serial.print(clat, 6); Serial.print(clong, 6); Serial.println(chead, 3);
       }
    }
      if(abs(wphead - chead) > headTol)
      {
        if((wphead - chead) < 0)
        {
          turnRight(3, 70);
        }
        else if ((wphead - chead) > 0)
        {
          turnLeft(3, 70);
        }
        else
        {
        }
        chead = getCurrentHeading();
        Serial.println("update 4 (head): "); Serial.println(chead, 3);
      }
    }

    count = count + 1;
  }
  else
  {
    delay(60000);
    count = 0;
  }

}

void moveForward(double time, double speed) //function to move forward given a time in seconds and a speed with a value between 0-127
{
  if (speed > 127) //conditional statement so speed stays within bounds
    {
      speed = 127;
    } 
    else if (speed < 0)
    {
      speed = 0;
    }
    else
    {
      speed = speed;
    }
  double start = millis(); //collects the current time in ms
  double begin = start; //records start time
  int timeGoal = (time * 1000); //converts given time to ms
  while (millis() <= timeGoal + start) //while loop to continue until time goal is met
  {
    roboclaw.ForwardM1(Front_Left,   speed);
    roboclaw.ForwardM1(Middle_Left,  speed);
    roboclaw.ForwardM1(Back_Left,    speed);
    roboclaw.BackwardM1(Back_Right,  speed); //right side is reversed because esc's don't understand orientation
    roboclaw.BackwardM1(Front_Right, speed);   
    roboclaw.BackwardM1(Middle_Right,speed);
  }
  double end = millis(); //collects current time in ms
  double total = (end - begin) / 1000; //calculates time taken for function to run in seconds
  Serial.print("Move forward time ---> ");   //prints move forward time
  Serial.println(total, 4); //prints calculated time to 4 decimal points
}

void turnRight(double time, double speed) //function to turn right given a time in seconds and a speed with a value between 0-127
{
  if (speed > 127) //conditional statement so speed stays within bounds
    {
      speed = 127;
    }
    else if (speed < 0)
    {
      speed = 0;
    }
    else
    {
      speed = speed;
    }
  double start = millis(); //collects the current time in ms
  double begin = start; //records start time
  int timeGoal = (time * 1000); //converts given time to ms
  while (millis() <= timeGoal + start) //while loop to continue until time goal is met
  {
    roboclaw.ForwardM1(Front_Left,   speed);
    roboclaw.ForwardM1(Middle_Left,  speed);
    roboclaw.ForwardM1(Back_Left,    speed);
    roboclaw.ForwardM1(Back_Right,   speed); //right side is reversed because esc's don't understand orientation
    roboclaw.ForwardM1(Front_Right,  speed);   
    roboclaw.ForwardM1(Middle_Right, speed);
  }
  double end = millis(); //collects current time in ms
  double total = (end - begin) / 1000; //calculates time taken for function to run in seconds
  Serial.print("turn right time ---> "); //prints turn right time
  Serial.println(total, 4);   //prints total function time to 4 decimal points
}

void turnLeft(double time, double speed)  //function to turn left given a time in seconds and a speed with a value between 0-127
{
  if (speed > 127)  //conditional statement so speed stays within bounds
    {
      speed = 127;
    }
    else if (speed < 0)
    {
      speed = 0;
    }
    else
    {
      speed = speed;
    }
  double start = millis(); //collects the current time in ms
  double begin = start; //records start time
  int timeGoal = (time * 1000); //converts given time to ms
  while (millis() <= timeGoal + start) //while loop to continue until time goal is met
  {
    roboclaw.BackwardM1(Front_Left,   speed);
    roboclaw.BackwardM1(Middle_Left,  speed);
    roboclaw.BackwardM1(Back_Left,    speed);
    roboclaw.BackwardM1(Back_Right,   speed); //right side is reversed because esc's don't understand orientation
    roboclaw.BackwardM1(Front_Right,  speed);   
    roboclaw.BackwardM1(Middle_Right, speed);
  }
  double end = millis(); //collects current time in ms
  double total = (end - begin) / 1000; //calculates time taken for function to run in seconds
  Serial.print("turn left time ---> "); //prints turn left time
  Serial.println(total, 4); //prints total function time with 4 decimal points
}

void stop(double time) //stops the robot, given a value in seconds
{
  double start = millis(); //collects the current time in ms
  double begin = start; //records start time
  int timeGoal = (time * 1000); //converts given time to ms
  while (millis() <= timeGoal + start) //while loop to continue until time goal is met
  {
    roboclaw.ForwardBackwardM1(Front_Left,  64);
    roboclaw.ForwardBackwardM1(Front_Right, 64);
    roboclaw.ForwardBackwardM1(Back_Left,   64);
    roboclaw.ForwardBackwardM1(Back_Right,  64);    
    roboclaw.ForwardBackwardM1(Middle_Left, 64);
    roboclaw.ForwardBackwardM1(Middle_Right,64);
  }
  double end = millis(); //collects current time in ms
  double total = (end - begin) / 1000; //calculates time taken for function to run in seconds
  Serial.print("stop time ---> "); //prints stop time
  Serial.println(total, 4);  //prints total function time with 4 decimal points
}

double getCurrentHeading()
{
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.println("cheading function ran successfully");
  
  return heading;
}

double haversine(double lat1, double lng1, double lat2, double lng2)
{
  double pi = M_PI;
  double earthR = 6371000;
  double x = pow(sin(((lat2-lat1)*pi / 180)/2),2);
  double y = cos(lat1 * pi / 180) * cos(lat2 * pi / 180);
  double z = pow(sin(((lng2 - lng2)*pi/180)/2), 2);
  double a = x + y * z;
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
  double d = earthR * c;
  return d;
}

double latitude()
{
  float lat = GPS.latitude;
      int degrees = (lat / 100);
      float minutes = lat - (degrees*100);
      minutes = minutes / 60;
      float degreeval = degrees + minutes;
      char direction = GPS.lat;
      if (direction == 'N')
      {
        degreeval = degreeval;
      }
      if (direction == 'S')
      {
        degreeval = degreeval * -1;
      }
      return degreeval;
}

double longitude()
{
  float lng = GPS.longitude;
    int degrees = (lng / 100);
    float minutes = lng - (degrees*100);
    minutes = minutes / 60;
    float degreeval = degrees + minutes;
    char direction = GPS.lon;
    if (direction == 'E')
    {
      degreeval = degreeval;
    }
    if (direction == 'W')
    {
      degreeval = degreeval * -1;
    }
    return degreeval; 
}

