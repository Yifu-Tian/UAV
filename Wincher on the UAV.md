# Abstract
## Winch System
&emsp;The winch system on UAV consists of a support base, a motor with a reducer, a winch and a tethered metal block.
The winch system can be easily strapped to the abdomen of various UAVs and it is reponsible for two tasks.  
&emsp;First, the winch system releases the metal block which is like a "decoy" to the magnetic catcher to establish a connection.  
&emsp;Second, it also provides the driving forces to enable UAV declining until touching down the landing pad.
## Program
```C
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

// WiFi Definitions //

#define PIN_INPUT 15      //  
#define PWM_PIN 26
#define Brake_pin 5
const char* ssid = "114514";
const char* password = "114514";

IPAddress server(1, 1, 4, 514);  //  IbP ros server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;

int  M1_IN1 = 2;
int  M1_IN2 = 4;

int  Motor_Rise_Flag = 0;
int  Motor_Drop_Flag = 0;
int  Motor_Stop_Flag = 0;
int  Delay_Stop_Flag = 0;

class WiFiHardware {
  public:
    WiFiHardware() {};

    void init() {
      // do your initialization here. this probably includes TCP server/client setup
      client.connect(server, 11411);
    }

    // read a byte from the serial port. -1 = failure
    int read() {
      // implement this method so that it reads a byte from the TCP connection and returns it
      //  you may return -1 is there is an error; for example if the TCP connection is not open
      return client.read();         //will return -1 when it will works
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length) {
      // implement this so that it takes the arguments and writes or prints them to the TCP connection
      for (int i = 0; i < length; i++)
        client.write(data[i]);
    }

    // returns milliseconds since start of program
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

int i;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to"); Serial.println(ssid);
    while (1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

// ---------------------
void chatterCallback(const std_msgs::String& msg)
{
// in ros terminal: rostopic pub message std_msgs/String "data:123"
  Serial.println(msg.data);
  String prompt = msg.data;

  if ( prompt == "drop")
  {
    Motor_Drop_Flag = 1;
    Motor_Rise_Flag = 0;
    Motor_Stop_Flag = 0;
    Delay_Stop_Flag = 1;
  }
  if ( prompt == "rise")
  {
    Motor_Drop_Flag = 0;
    Motor_Rise_Flag = 1;
    Motor_Stop_Flag = 0;
    Delay_Stop_Flag = 1;
  }
  if ( prompt == "stop")
  {
    Motor_Drop_Flag = 0;
    Motor_Rise_Flag = 0;
    Motor_Stop_Flag = 1;
    Delay_Stop_Flag = 0;
  }

}


std_msgs::String str_msg;

ros::Publisher chatter("wincher_chatter", &str_msg);

ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);

ros::NodeHandle_<WiFiHardware> nh;

void Motor_Drop(void)
{
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(Brake_pin,LOW);
  Serial.print("Dropping the block...");
}

void Motor_Rise(void)
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(Brake_pin, HIGH); // This is important
  Serial.print("Rising the block...");
}

void Motor_Stop(void)
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  Serial.print("Stop");
}

void setup() {
  Serial.begin(115200);
  Serial.print("Wincher start\n");
  Serial.print("test rise\n");
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, HIGH);

  pinMode(Brake_pin, OUTPUT);
  setupWiFi();
  delay(2000);
  nh.initNode();

  nh.advertise(chatter);
  nh.subscribe(sub);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(PIN_INPUT, INPUT_PULLUP);
  

}

void loop() {
  nh.spinOnce();
  delay(500);

  if (Motor_Rise_Flag == 1 && digitalRead(PIN_INPUT) == HIGH) 
  // not contact
  {
    Motor_Rise();
    delay(4500);
    Motor_Stop();
    Motor_Rise_Flag = 0;
  }

  else if (Motor_Drop_Flag == 1 ) {
    Motor_Drop();
    delay(4500);
    Motor_Stop();
    Motor_Drop_Flag = 0;
  }
  else if (Delay_Stop_Flag == 1 && digitalRead(PIN_INPUT) != HIGH) 
  // contact
  {
    Delay_Stop_Flag = 0;
    delay(1500);
    Motor_Stop();
  }
  else if (Motor_Stop_Flag == 1) {
    Motor_Stop();
  }

}
```
# Remark
&emsp;Well, I do agree the code looks tedious. Maybe I will optimize the code from time to time.
