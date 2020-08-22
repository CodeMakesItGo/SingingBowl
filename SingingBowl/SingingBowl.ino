/*Developed by M V Subrahmanyam - https://www.linkedin.com/in/veera-subrahmanyam-mediboina-b63997145/
Project: AWS | NodeMCU ESP32 Tutorials
Electronics Innovation - www.electronicsinnovation.com

GitHub - https://github.com/VeeruSubbuAmi
YouTube - http://bit.ly/Electronics_Innovation

Upload date: 07 October 2019

AWS Iot Core

This example needs https://github.com/esp8266/arduino-esp8266fs-plugin

It connects to AWS IoT server then:
- publishes "hello world" to the topic "outTopic" every two seconds
- subscribes to the topic "inTopic", printing out any messages
*/

#include "FS.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
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
#define MAX_DELAY 5000     //Max time delay for stepper motor
#define RAMP_RATE 50        //time for each step in milliseconds
#define SERVO_PARK 70       //default position for servo

const char* ssid = "CenturyLink4216";
const char* password = "8qzH1AMsY4QzG9hD";
const char* AWS_endpoint = "a3m10fn05mfcbt-ats.iot.us-west-2.amazonaws.com"; //MQTT broker ip

Servo servo;                //Servo object to control the servo PWM
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
WiFiClientSecure espClient;
void callback(char* topic, byte* payload, unsigned int length);
PubSubClient client(AWS_endpoint, 8883, callback, espClient); //set MQTT port number to 8883 as per //standard

//MQTT global data
long lastMsg = 0;
char msg[50];
int value = 0;

typedef struct playset
{
  bool fast_step;       //servo move fast or controlled
  short servo_pos;      //servo pos goal (deg)
  short motor_delay;    //motor delay in us
  long milli_sec;       //milisecond time
};

short servo_pos = SERVO_PARK;   //Global servo position in degrees
short motor_delay = MAX_DELAY;  //Global stepper motor time delay in microseconds
byte playset_index = 0;         //Current index into the playset
const byte playset_max = 6;     //number of play instructions in the playset
bool playing = false;           //flag to indicate if the playset is playing

                       // fast  pos           delay         msec    
playset thePlayset[] = {  {0,   SERVO_PARK,   MAX_DELAY,    5000  },  //Start at a stopped position
                          {1,   50,           2500,         100   },  //quickly move to strike the bowl
                          {1,   SERVO_PARK,   2500,         1000  },  //quickly move to resting position and start to rotate bowl
                          {0,   30,           2500,         60000 },  //slowly move stiker in place
                          {0,   40,           3000,         10000 },  //start to slow down
                          {0,   SERVO_PARK,   MAX_DELAY,    5000  }}; //move to stop and repeat

/// The MQTT callback
/// INPUTS: topic =  the topic name received
///         payload = the data the topic contains
///         length = the legth of the payload bytes
/// OUTPUTS: none
void callback(char* topic, byte* payload, unsigned int length) 
{
  playing =  (char)payload[7] == '1';

  if(playing)
  {
    Serial.println("Start");
  }
  else
  {
    Serial.println("Stop");
  }
}

/// Connect to the WIFI access point
/// INPUTS: none
/// OUTPUTS: none
void setup_wifi() 
{
  delay(10);
  // We start by connecting to a WiFi network
  espClient.setBufferSizes(512, 512);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  while(!timeClient.update())
  {
    timeClient.forceUpdate();
  }

  espClient.setX509Time(timeClient.getEpochTime());
}

/// Reconnect to the MQTT if the connection is lost
/// INPUTS: none
/// OUTPUTS: none
void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESPthing")) 
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("get", "hello world");
      // ... and resubscribe
      client.subscribe("set");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      
      char buf[256];
      espClient.getLastSSLError(buf,256);
      Serial.print("WiFiClientSecure SSL error: ");
      Serial.println(buf);
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/// Default setup called by Arduino framework
/// Inputs: none
/// Outpus: none
void setup() 
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
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
    
  setup_wifi();
  delay(1000);
  
  if (!SPIFFS.begin()) 
  {
    Serial.println("Failed to mount file system");
    return;
  }

  // Load certificate file
  File cert = SPIFFS.open("/cert.der", "r"); //replace cert.crt eith your uploaded file name
  if (!cert) 
    Serial.println("Failed to open cert file");
  else
    Serial.println("Success to open cert file");
  
  delay(1000);

  if (espClient.loadCertificate(cert))
    Serial.println("cert loaded");
  else
    Serial.println("cert not loaded");
  
  // Load private key file
  File private_key = SPIFFS.open("/private.der", "r"); //replace private eith your uploaded file name
  if (!private_key) 
    Serial.println("Failed to open private cert file");
  else
    Serial.println("Success to open private cert file");
  
  delay(1000);

  if (espClient.loadPrivateKey(private_key))
    Serial.println("private key loaded");
  else
    Serial.println("private key not loaded");
  
  // Load CA file
  File ca = SPIFFS.open("/ca.der", "r"); //replace ca eith your uploaded file name
  if (!ca) 
    Serial.println("Failed to open ca ");
  else
    Serial.println("Success to open ca");
  
  delay(1000);

  if(espClient.loadCACert(ca))
    Serial.println("ca loaded");
  else
    Serial.println("ca failed");
  
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
}

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

/// The main loop
/// Inputs: none
/// Outputs: none
void loop() 
{
  static long current_msec = millis();
  
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();
  
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
