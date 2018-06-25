/*
 * ROS Differential Drive Robot Controller for Teensy 3.X
 * We define that the robot moves forward when the left motor moves CCW
 * when viewed from the shaft extension end and the right motor moves CW.
 * If the motors run in the opposite way, reverse the motor wires to reverse rotation.
 */

#define USE_USBCON // eliminates sync error of serial_node with a Teensy
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// For mapping admissible input values to admissible serial command values
float cmdMin = -0.3; // min addmisible input command value [mt/s]
float cmdMax = 0.3; // max addmisible input command value [mt/s]
float toLow = -63; // min addmisible serial value for Sabertooth [int]
float toHigh = 63; // max addmisible serial value for Sabertooth [int]

ros::NodeHandle  nh;

float pi = 3.1415;
float L = 0.1; //distance between wheels

void cmdLeftWheelCB( const std_msgs::Int16& msg)
// Read from topic cmd_left_wheel 
// The command signals are expected to be between -63 and 63.
// Example: $ rostopic pub /cmd_left_wheel std_msgs/Int16 "data: 30
// Motors should stop with a zero value.ue.
{
  Serial2.write(setmotor(1, msg.data));   // move left motor 
}

void cmdRightWheelCB( const std_msgs::Int16& msg)
// Read from topic cmd_right_wheel 
// The command signals are expected to be between -63 and 63.
// Example: $ rostopic pub /cmd_right_wheel std_msgs/Int16 "data: 30
// Motors should stop with a zero value.
{
    Serial2.write(setmotor(2, msg.data));   // move right motor 
}

ros::Subscriber<std_msgs::Int16> subCmdLeft("cmd_left_wheel", cmdLeftWheelCB );
ros::Subscriber<std_msgs::Int16> subCmdRight("cmd_right_wheel",cmdRightWheelCB );


void cmdVelCB( const geometry_msgs::Twist& twist)
// Read from topic cmd_vel 
// The command signals are expected to be between -0.3 and 0.3 [mts/s]
// Example: $ rostopic pub /cmd_vel geometry_msgs/Twist -r 3 -- '[0.2,0.0,0.0]' '[0.0, 0.0, 0.0]'
// Motors should stop with a zero value.
{
  int gain = 1;
  float left_wheel_data = gain*(twist.linear.x - twist.angular.z*L);
  float right_wheel_data = gain*(twist.linear.x + twist.angular.z*L);
  moveLeftMotor(left_wheel_data);  // move left motor
  moveRightMotor(right_wheel_data);  // move right motor
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

void setup() 
{
  Serial2.begin(9600); // initialize serial2 port, baud rate must match DIP switches 
  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
}

void loop() 
{ 
    nh.spinOnce();  
}

void moveLeftMotor(float leftMsValue)
{
  if (leftMsValue >= cmdMax) {
    leftMsValue = cmdMax; // speed cap
  }
  else if (leftMsValue <= cmdMin) {
          leftMsValue = cmdMin; // speed cap
  }
  // map mt/s speed value to a valid value for simple serial motor input 
  double leftMsScaled = mapf(leftMsValue, cmdMin, cmdMax, toLow, toHigh);
  int leftCmdSerial = int(leftMsScaled); // typecast
  Serial2.write(setmotor(1, leftCmdSerial));   // move left motor
}

void moveRightMotor(float rightMsValue)
{
  if (rightMsValue >= cmdMax) {
    rightMsValue = cmdMax; // speed cap
  }
  else if (rightMsValue <= cmdMin) {
          rightMsValue = cmdMin; // speed cap
  }
  // map mt/s speed value to a valid value for simple serial motor input 
  double rightMsScaled = mapf(rightMsValue, cmdMin, cmdMax, toLow, toHigh);
  int rightCmdSerial = int(rightMsScaled); // typecast
  Serial2.write(setmotor(2, rightCmdSerial));  // move right motor
}

byte setmotor(byte motor, int power)
/*
Because Sabertooth controls two motors with one 8 byte character, when operating in Simplified
Serial mode, each motor has 7 bits of resolution. Sending a character between 1 and 127 will
control motor 1. 1 is full reverse, 64 is stop and 127 is full forward. Sending a character between
128 and 255 will control motor 2. 128 is full reverse, 192 is stop and 255 is full forward.
Sending a value of 0 will shut down both motors.
*/
{
  byte command = 0;
  power = constrain(power, -63, 63);

  if (motor == 1)
  {
    command = 64 + power;
  }
  else if (motor == 2)
  {
    command = 191 + power ;
  }
  return command;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
// Arduino map function for float values
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
