#include <SoftwareSerial.h>
#include <RoboClaw.h>


SoftwareSerial serial(16,17); // rx, tx pins on esp32   //16==s2   //17==s1
RoboClaw roboclaw(&Serial2,10000);

#define Front_Left  0x80 // Address for the Front Left motor
#define Front_Right 0x81 // Address for the Front Right motor
#define Back_Left   0x82 // Address for the Back Left motor
#define Back_Right  0x83 // Address for the Back Right motor
#define Middle_Left 0x85 // Address for the Middle Left motor
#define Middle_Right 0x84 // Address for the Middle Right motor


void accelerate(int time, int startSpeed, int endSpeed)
{
  double start = millis();
  time = time*1000;
  double steps = time / 185;
  double difference = endSpeed - startSpeed;
  double speedIncrease = difference / steps ;
  double currentSpeed = startSpeed - speedIncrease;
  int count = 1;
  while (millis() < (time + start))
  {
    currentSpeed = currentSpeed + speedIncrease;
    if (currentSpeed > 127)
    {
      currentSpeed = 127;
    }
    else if (currentSpeed < 0)
    {
      currentSpeed = 0;
    }
    else
    {
      currentSpeed = currentSpeed;
    }
    double ls = millis();
    roboclaw.ForwardM1(Front_Left,    currentSpeed);
    roboclaw.ForwardM1(Middle_Left,   currentSpeed);
    roboclaw.ForwardM1(Back_Left,     currentSpeed);
    roboclaw.BackwardM1(Back_Right,   currentSpeed); 
    roboclaw.BackwardM1(Front_Right,  currentSpeed);   
    roboclaw.BackwardM1(Middle_Right, currentSpeed);
    
    Serial.print("Stage:");
    Serial.print(count);
    Serial.print("  ");
    if (count < 10)
    {
      Serial.print(" ");
    }
    Serial.print("Current speed -> ");
    Serial.print(currentSpeed, 5);
    Serial.print("    ");
    if (currentSpeed < 10)
    {
      Serial.print("  ");
    }
    else if ((currentSpeed > 10) && (currentSpeed < 100))
    {
      Serial.print(" ");
    }
    double le = millis();
    double lt = le - ls;
    Serial.print("acc. loop time ---> ");
    Serial.println(lt, 4);
    count++;
  }
  double end = millis();
  double total = (end - start) / 1000;
  Serial.print("Accelerate time ---> ");
  Serial.println(total, 4);
  Serial.println("");
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

void moveBackward(double time, double speed) //function to move backwards given a time in seconds and a speed with a value between 0-127
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
    roboclaw.BackwardM1(Front_Left,   speed);
    roboclaw.BackwardM1(Middle_Left,  speed);
    roboclaw.BackwardM1(Back_Left,    speed);
    roboclaw.ForwardM1(Back_Right,    speed); //right side is reversed because esc's don't understand orientation
    roboclaw.ForwardM1(Front_Right,   speed);   
    roboclaw.ForwardM1(Middle_Right,  speed);
  }
  double end = millis(); //collects current time in ms
  double total = (end - begin) / 1000; //calculates time taken for function to run in seconds
  Serial.print("Move backwards time ---> ");   //prints move backwards time
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

void setup() {
  // put your setup code here, to run once:
  roboclaw.begin(38400); // This is the baudrand that was set in the basicmicro motion studio. Make sure that all motor drivers have the same baudrand.
  Serial.begin(9600);
  Serial.println("");
  Serial.println("#####################################################################################");
  stop(1);

  //adjustable
  accelerate(4, 0, 127);
  moveForward(3, 127);
  accelerate(4, 127, 0);
  //moveForward(4, 100);
  stop(1);
  moveBackward(2, 50);
  stop(1);
  turnLeft(4, 90);
  //stop(1);
  //turnRight(3, 120);



  stop(1);
  Serial.println("Program finished");
  Serial.println("#####################################################################################");
}

void loop() {
 
}
