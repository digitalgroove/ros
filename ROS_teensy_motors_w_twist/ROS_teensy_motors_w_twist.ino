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

#include <std_msgs/Float32.h> // used to publish wheel velocities
#include <TimerOne.h>
#define LOOP_TIME 200000 // set frecuency to 5 Hz

#define left_encoder_pin 2 // define encoder pins
#define right_encoder_pin 3

unsigned int counter_left = 0; // initialize encoder counters
unsigned int counter_right = 0;

// For mapping admissible input values to admissible serial command values
float cmdMin = -0.3; // min addmisible input command value [mt/s]
float cmdMax = 0.3; // max addmisible input command value [mt/s]
float toLow = -63; // min addmisible serial value for Sabertooth [int]
float toHigh = 63; // max addmisible serial value for Sabertooth [int]

// constants for kinematic equations
#define PII 3.1415
#define AXIS 0.385 //distance between wheels in meters
#define RADIUS 0.0508 //wheel radius in meters (2 inch)
#define TICKS 2048 // encoder pulses per rotation 2x decoding

ros::NodeHandle  nh;

// For debugging: Publish serial command send to motor driver
std_msgs::Int16 left_wheel_serial_cmd;  // variable declaration
ros::Publisher left_wheel_serial_cmd_pub("/left_wheel_serial_cmd", &left_wheel_serial_cmd);
std_msgs::Int16 right_wheel_serial_cmd;  // variable declaration
ros::Publisher right_wheel_serial_cmd_pub("/right_wheel_serial_cmd", &right_wheel_serial_cmd);

// Initialize variables used to publish wheel velocities
std_msgs::Float32 left_wheel_vel; // variable declaration
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);
std_msgs::Float32 right_wheel_vel; // variable declaration
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

// helper functions for encoder counts
void docount_left()  // counts from the speed sensor
{
  counter_left++;  // increase +1 the counter value
} 

void docount_right()  // counts from the speed sensor
{
  counter_right++;  // increase +1 the counter value
}

void timerIsr()
// hardware timer to publish wheel velocity messages 
{
  Timer1.detachInterrupt();  //stop the timer
  // Left Motor Speed
  // wheel circumference / counts per revolution = distance traveled per encoder count
  // velocity = (wheel circumference / counts per revolution) / time
  // Since RADIUS is in mt and Ticks is in 1/second then both are in meters/second:
  left_wheel_vel.data = float(counter_left)*((2*PII*RADIUS)/TICKS)*5; // counts at 5 Hz x 5 to get counts x sec
  left_wheel_vel_pub.publish(&left_wheel_vel); // publishes in mt/sec
  right_wheel_vel.data = float(counter_right)*((2*PII*RADIUS)/TICKS)*5; // counts at 5 Hz x 5 to get counts x sec
  right_wheel_vel_pub.publish(&right_wheel_vel); // publishes in mt/sec
  counter_right=0;
  counter_left=0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void cmdLeftWheelCB( const std_msgs::Int16& msg)
// Read from topic cmd_left_wheel 
// The command signals are expected to be between -63 and 63.
// Example: $ rostopic pub /cmd_left_wheel std_msgs/Int16 "data: 30
// Motors should stop with a zero value.
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
// The linear speed command signals are expected to be between -0.3 and 0.3 [mts/s]
// Example: $ rostopic pub /cmd_vel geometry_msgs/Twist -r 3 -- '[0.2,0.0,0.0]' '[0.0, 0.0, 0.0]'
// Motors should stop with a zero value.
{
  int gain = 1;
  // unicycle to differential drive equations
  // linear.x = forward velocity (mt/sec), angular.z = angular velocity (rad/sec)
  // angular velocity of wheels in rad/sec, counter-clock-wise and clock-wise respectively:
  float left_wheel_radians = gain * (2 * twist.linear.x - twist.angular.z * AXIS) / (2 * RADIUS);
  float right_wheel_radians = gain * (2 * twist.linear.x + twist.angular.z * AXIS) / (2 * RADIUS);
  // transform angular velocities (rad/sec) to linear velocities (mt/sec):
  float left_wheel_data = left_wheel_radians * RADIUS;
  float right_wheel_data = right_wheel_radians * RADIUS;
  moveLeftMotor(left_wheel_data);  // move left motor
  moveRightMotor(right_wheel_data);  // move right motor
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

void setup() 
{
  Serial2.begin(9600); // initialize serial2 port, baud rate must match DIP switches 
  //Setup for encoders
  pinMode(right_encoder_pin, INPUT); // No pull-up resistors needed
  pinMode(left_encoder_pin, INPUT);
  Timer1.initialize(LOOP_TIME);
  attachInterrupt(digitalPinToInterrupt(left_encoder_pin), docount_left, CHANGE); // increase counter when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), docount_right, CHANGE); // increase counter when speed sensor pin changes
  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
  // Publish serial command send to motor driver 
  nh.advertise(left_wheel_serial_cmd_pub);
  nh.advertise(right_wheel_serial_cmd_pub);
  // Publish speed of wheels
  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);
  Timer1.attachInterrupt( timerIsr ); // enable the timer
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
  //Publish serial command send to motor driver 
  left_wheel_serial_cmd.data = leftCmdSerial;
  left_wheel_serial_cmd_pub.publish(&left_wheel_serial_cmd);
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
  //Publish serial command send to motor driver 
  right_wheel_serial_cmd.data = rightCmdSerial;
  right_wheel_serial_cmd_pub.publish(&right_wheel_serial_cmd);  
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
