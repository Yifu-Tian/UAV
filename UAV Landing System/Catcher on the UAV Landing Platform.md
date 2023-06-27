# Abstract
&emsp;The catcher mainly consists of a magnets array, a circular support base, an aligning linkage mechanism, and one servo motor.  
![1](https://github.com/TTrravis/ROS_learning/blob/main/pic/正向视野.png)  
&emsp;Once a metal block enters into the vicinity of this magnetic area, it would be attracted firmly. Afterward, the aligning mechanism would start to shrink into a reduced state. ![2](https://github.com/TTrravis/ROS_learning/blob/main/pic/张开俯视.png)  
&emsp;This is an alignment process in which the metal block would be forced to move to the center of the support base.
![3](https://github.com/TTrravis/ROS_learning/blob/main/pic/收紧俯视.png)  
# Program
&emsp;You may refer to my other passages to get more help:
[Wireless connection of ROS with multiple ESP32](https://github.com/TTrravis/ROS_learning.git)

```C
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>


// WiFi Definition
#define PIN_INPUT 15

const char* ssid = "114514";
const char* password = "114514";
 
IPAddress server(11, 4, 5, 14); 
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client; // declare a client, used for connection with sever

class WiFiHardware {
  public:
  WiFiHardware() {};
 
  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server,11411);
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
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
  
  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

//-------------------

int Catcher_Align_Flag=0;
int Catcher_Open_Flag =0;

char Speed_Off[8] =  {0x80,0x06,0x00,0x40,0x00,0x00,0x96,0x0F}; //停止
char Speed_Align[8]= {0x80,0x06,0x00,0x40,0x03,0xe7,0xd6,0xb5}; //正转
char Speed_Open[8] = {0x80,0x06,0x00,0x40,0xfc,0x19,0x16,0xc5}; //饭转 


int i;
 
void chatterCallback(const std_msgs::String& msg) 
{
  // first print out what the subscriber function sub() recieved
  Serial.println(msg.data);
  String prompt = msg.data;
  if( prompt == "align")
  {
    Serial.print("Ready to align.");
    Catcher_Align_Flag =1;
    Catcher_Open_Flag = 0;
  }
  if( prompt == "open")
  {      
    Serial.print("Ready to open.");
    Catcher_Align_Flag = 0;
    Catcher_Open_Flag = 1;
  }
}
 
 
std_msgs::String str_msg;
std_msgs::String str_msg2;                             

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher catcher_commander("message",&str_msg2);    

ros::Subscriber<std_msgs::String> sub("message2", &chatterCallback); // rostopic pub "message2" in the terminal(align/open)

ros::NodeHandle_<WiFiHardware> nh;
char hello[20] = "Catcher alive!";
char catcher_align[20] = "Catcher align!"; 
char catcher_open[20]  = "Catcher open!!";
char catcher_command[7] = "upload";

void Catcher_Align()
{
  for(int i =0;i<8;i++){
    Serial2.write(Speed_Align[i]); // Take mind Serial2 here!
   }
}

void Catcher_Open()
{
   for(int i =0;i<8;i++){
    Serial2.write(Speed_Open[i]);
   }
}

void Catcher_MotorOff()
{
   for(int i =0;i<8;i++){
    Serial2.write(Speed_Off[i]);
   }  
}

void setup() 
{
  Serial.begin(115200);
  Serial.print("Begin");
  Serial2.begin(9600,SERIAL_8E1);
  setupWiFi();
  delay(2000);

  nh.initNode();

  nh.advertise(catcher_commander);
  
  nh.advertise(chatter);
  
  nh.subscribe(sub);

  Catcher_Open_Flag = 0;
  
  pinMode(PIN_INPUT, INPUT_PULLUP);
 
}
 
void loop() 
{
  nh.spinOnce();
  delay(20);
 

  
//---------------------------------------------------------------
  if(Catcher_Align_Flag == 1 && digitalRead(PIN_INPUT)!= HIGH)
  {  
    // contact
    Catcher_Align_Flag = 0;
    
    Catcher_Align();
    
    delay(1500);
    
    Catcher_MotorOff();
    
    Serial.printf("Align");
    
    str_msg.data = catcher_align;  // = "catcher align"
    
    chatter.publish(&str_msg);

    str_msg2.data = catcher_command;    // ="upload"

    catcher_commander.publish(&str_msg2); 
  } 
  else if(Catcher_Open_Flag ==1 && digitalRead(PIN_INPUT)!= LOW)
  {
    // if receive the open command
    Catcher_Open_Flag = 0;
    
    Catcher_Open();
    
    delay(1500);
    
    Catcher_MotorOff();
    
    Serial.printf("Open!!!");
    
    str_msg.data = catcher_open; //="catcher open"
    
    chatter.publish(&str_msg);
   }

}
```
# Launch 
1. Enter `roscore` in the terminal  
2. Enter `rosrun rosserial_python serial_node.py tcp`  
3. Enter `rostopic pub /message2 std_msgs/String align` to start aligning.  
4. Enter `rostopic pub /message2 std_msgs/String open` to open.  
