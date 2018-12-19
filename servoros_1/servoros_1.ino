/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;
std_msgs::Float64 message;
Servo servo;
float x;
void servo_cb(const std_msgs::Float64& cmd_msg){
message.data=cmd_msg.data;
x = message.data;
}
std_msgs::Float64 test;
ros::Subscriber<std_msgs::Float64> sub("needle_locate", servo_cb);
ros::Publisher p("my_topic", &test);


void setup(){
pinMode(13, OUTPUT);

nh.initNode();
nh.subscribe(sub);
nh.advertise(p);

servo.attach(9); //attach it to pin 9
}

bool flag=false;
void loop(){
if(!flag)
{  
servo.writeMicroseconds(1500);
delay(3000);
flag=true;
}

else {

  servo.writeMicroseconds(message.data);  // set servo angle, should be from 0-180  
  test.data = x;
  p.publish( &test );
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

  nh.spinOnce();
  delay(1);  
}
      nh.spinOnce();
}
