/*  This is the sound bowl application for the sound bowl platform
 *  This program has been developed for the Hackaday.io "Change the Planet with PSoC® IoT Design" challenge
 *  The project details are located here: https://hackaday.io/project/171713-singing-bowl-player
 *  Developed by W. Jason Altice 
 *  Copyright 2020
 */

#include <Servo.h> 

//Stepper motor modes
#define HALF_STEP 1         //slowest but highest torque
#define FULL_STEP 2         //medium speed medium torque
#define DOUBLE_STEP 3       //fast speed low torque
#define STEP_MODE FULL_STEP //Set to FULL Stepping

//Movement stepping
#define MOTOR_STEP 100      //change delay time 100us per step
#define SERVO_STEP 1        //Change servo degrees per step

//Default values
#define MAX_DELAY 20000     //Max time delay for stepper motor
#define RAMP_RATE 50        //time for each step in milliseconds
#define SERVO_PARK 70       //default position for servo

//The playset is a set of instructions on how to play the bowl
typedef struct 
{
  bool fast_step;       //servo move fast or controlled
  short servo_pos;      //servo pos goal (deg)
  short motor_delay;    //motor delay in us
  long milli_sec;       //milisecond time
} playset;

//Servo object to control the servo PWM
Servo servo;

short servo_pos = SERVO_PARK;   //Global servo position in degrees
short motor_delay = MAX_DELAY;  //Global stepper motor time delay in microseconds
byte playset_index = 0;         //Current index into the playset
const byte playset_max = 6;     //number of play instructions in the playset
bool playing = false;           //flag to indicate if the playset is playing

                       // fast  pos           delay         msec    
playset thePlayset[] = {  {0,   SERVO_PARK,   MAX_DELAY,    5000  },  //Start at a stopped position
                          {1,   50,           MAX_DELAY,    500   },  //quickly move to strike the bowl
                          {1,   SERVO_PARK,   2500,         5000  },  //quickly move to resting position and start to rotate bowl
                          {0,   30,           2500,         60000 },  //slowly move stiker in place
                          {0,   40,           5000,         10000 },  //start to slow down
                          {0,   SERVO_PARK,   MAX_DELAY,    5000  }}; //move to stop and repeat


/// The servoOutput will control the servo position
/// Inputs: none
/// Outputs: none
void servoOutput(short goal)
{
  static long last_update = millis();

  //Run ramp 
  if(millis() - last_update > RAMP_RATE)
  {
    last_update = millis();
       
    //if fast step move servo to position
    if(thePlayset[playset_index].fast_step)
    {
       servo.write(goal);
    }
    else
    {
      if(goal > servo_pos)
      {
        if(goal <= servo_pos + SERVO_STEP)
        {
          servo_pos = goal;
        }
        else
        {
          servo_pos += SERVO_STEP;
        }
      }
      
      if(goal < servo_pos)
      {
        if(goal >= servo_pos - SERVO_STEP)
        {
          servo_pos = goal;
        }
        else
        {
          servo_pos -= SERVO_STEP;
        }
      }
  
      servo.write(servo_pos);
    }
  }
}

/// The stepperOutput will control the stepper motor according to the step mode and speed
/// Inputs: none
/// Outputs: none
void stepperOutput(short goal)
{
  static int output = 0;
  static long last_update = millis();

  //Run ramp 
  if(millis() - last_update > RAMP_RATE)
  {
    last_update = millis();
    
    //Do we need to speed up?
    if(goal > motor_delay)
    {
      //Are we close to goal?
      if(goal <= motor_delay + MOTOR_STEP)
      {
        motor_delay = goal;
      }
      else
      {
        motor_delay += MOTOR_STEP;
      }
    }
  
    //Do we need to slow down?
    if(goal < motor_delay)
    {
       //Are we close to goal?
      if(goal >= motor_delay - MOTOR_STEP)
      {
        motor_delay = goal;
      }
      else
      {
        motor_delay -= MOTOR_STEP;
      }
    }
  }

  if(motor_delay == MAX_DELAY)
  {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    digitalWrite(D4, LOW);
  }
  else
  {
    //Limit to range 0 - 7
    output = output % 8;
  
    //enable output based on counter and step type
    digitalWrite(D1, output == 7 || output == 0 || output == 1 ? HIGH : LOW);
    digitalWrite(D2, output == 1 || output == 2 || output == 3 ? HIGH : LOW);
    digitalWrite(D3, output == 3 || output == 4 || output == 5 ? HIGH : LOW);
    digitalWrite(D4, output == 5 || output == 6 || output == 7 ? HIGH : LOW);

    //Using Full Steps
    output += STEP_MODE;
  }
  
  delayMicroseconds(motor_delay);
 
}

/// Default setup called by Arduino framework
/// Inputs: none
/// Outpus: none
void setup() 
{
  // Start the Serial comms
  Serial.begin(115200);         
  delay(10);
  Serial.println('\n');

  //Pin Setup to control the Stepper motor
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);

  //Pin setup to control the servo motor
  servo.attach(D5);
  servo.write(SERVO_PARK);
  delay(5000);
}

/// The main loop
/// Inputs: none
/// Outputs: none
void loop() 
{
  static long current_msec = millis();

  if(playing)
  {
    stepperOutput(thePlayset[playset_index].motor_delay);

    servoOutput(thePlayset[playset_index].servo_pos);

    //Test to go to the next play instruction index
    if((long)(millis() - current_msec) > thePlayset[playset_index].milli_sec)
    {
      playset_index += 1;
      playset_index %= playset_max;
      current_msec = millis();
    }
  }
  else
  {
    stepperOutput(MAX_DELAY);

    servoOutput(SERVO_PARK);

    playset_index = 0;
  } 
}
