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

#include <Encoder.h>
#include <std_msgs/Float32.h> // used to publish wheel velocities
#include <TimerOne.h>
#define LOOP_TIME 200000 // set frequency to 5 Hz

#define left_encoder_ChA 2 // define encoder pins
#define left_encoder_ChB 4

#define right_encoder_ChA 3
#define right_encoder_ChB 6

Encoder EncoderLeftWheel(left_encoder_ChA, left_encoder_ChB); // Create an Encoder object, using 2 pins.
Encoder EncoderRightWheel(right_encoder_ChA, right_encoder_ChB);

// output for pose estimation and input for get_new_heading_angle
// initial_pose = (0, 0, 0), values are x, y and orientation theta
// pose by wheel odometry could be initialized to a specific pose/orientation
float x_r = 0.0; // x coordinate value in meters
float y_r = 0.0; // y coordinate value in meters
float theta_r = 0.0; // robot orientation in WC, in rad [0,2*Pi)

// For mapping admissible input values to admissible serial command values
float cmdMin = -0.3; // min addmisible input command value [mt/s]
float cmdMax = 0.3; // max addmisible input command value [mt/s]
float toLow = -63; // min addmisible serial value for Sabertooth [int]
float toHigh = 63; // max addmisible serial value for Sabertooth [int]

// constants for kinematic equations
#define PII 3.1415
#define AXIS 0.385 //distance between wheels in meters
#define RADIUS 0.0508 //wheel radius in meters (2 inch)
#define TICKS 4096 // encoder pulses per rotation 4x decoding

/* ROS handler, instantiate with bigger buffer memory */
ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;

// For debugging: Publish serial command send to motor driver
//std_msgs::Int16 left_wheel_serial_cmd;  // variable declaration
//ros::Publisher left_wheel_serial_cmd_pub("/left_wheel_serial_cmd", &left_wheel_serial_cmd);
//std_msgs::Int16 right_wheel_serial_cmd;  // variable declaration
//ros::Publisher right_wheel_serial_cmd_pub("/right_wheel_serial_cmd", &right_wheel_serial_cmd);

// Initialize variables used to publish position (x,y)
std_msgs::Float32 x_pose; // variable declaration
ros::Publisher x_pose_pub("/x_pose", &x_pose);
std_msgs::Float32 y_pose; // variable declaration
ros::Publisher y_pose_pub("/y_pose", &y_pose);

geometry_msgs::Twist base_link_velocity;
ros::Publisher base_link_velocity_pub("/base_link_velocity", &base_link_velocity);

void timerIsr()
// hardware timer to publish wheel velocity messages
{
  Timer1.detachInterrupt();  //stop the timer
  // Calculate wheel speed
  // wheel circumference / counts per revolution = distance traveled per encoder count
  // velocity = (wheel circumference / counts per revolution) / time
  // Since RADIUS is in mt and Ticks is in 1/second then both are in meters/second:
  est_pose(&x_r, &y_r, &theta_r); // passing address-of
  x_pose.data = x_r; // x coordinate position
  x_pose_pub.publish(&x_pose); // publishes in mt/sec
  y_pose.data = y_r; // y coordinate position
  y_pose_pub.publish(&y_pose); // publishes in mt/sec
  // differential drive to unicycle equations:
  // mean velocity, the velocity of the center between the two wheels:
//  base_link_velocity.linear.x = (left_wheel_vel.data + right_wheel_vel.data)/2;
//  base_link_velocity.linear.y = 0;
//  base_link_velocity.linear.z = 0;
//  base_link_velocity.angular.x = 0;
//  base_link_velocity.angular.y = 0;
//  base_link_velocity.angular.z = (right_wheel_vel.data - left_wheel_vel.data)/AXIS; // ((rightTravel - leftTravel) / AXIS) / deltaTime
//  base_link_velocity_pub.publish(&base_link_velocity);
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
  Timer1.initialize(LOOP_TIME); // init timer for encoders
  nh.initNode();
  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
  // For debugging: Publish serial command send to motor driver
  // nh.advertise(left_wheel_serial_cmd_pub);
  // nh.advertise(right_wheel_serial_cmd_pub);
  // Publish speed of wheels
  nh.advertise(x_pose_pub);
  nh.advertise(y_pose_pub);
  nh.advertise(base_link_velocity_pub);
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
  // For debugging: Publish serial command send to motor driver
  // left_wheel_serial_cmd.data = leftCmdSerial;
  // left_wheel_serial_cmd_pub.publish(&left_wheel_serial_cmd);
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
  // For debugging: Publish serial command send to motor driver
  // right_wheel_serial_cmd.data = rightCmdSerial;
  // right_wheel_serial_cmd_pub.publish(&right_wheel_serial_cmd);
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


// Passes arguments by address, uses pointers to pass out values
void est_pose(float * x_rPtr, float * y_rPtr, float * theta_rPtr)
// get current coordinates with respect to initial position
// estimate_pose() called in the loop()
{

// 1. Save the current encoder ticks for each wheel
int current_L_ticks = EncoderLeftWheel.read();
int current_R_ticks = -1*EncoderRightWheel.read(); // sign change (R-motor turns CW, R-encoder CCW when robot moves forward)
// 2. Reset encoders to zero in order to start counting again
EncoderLeftWheel.write(0);
EncoderRightWheel.write(0);
// 3. Calculate dt distance = wheel circumference*counts/counts per revolution
float mt_per_tick = (2.0 * PII * RADIUS) / TICKS;
// distance travelled by left wheel and right wheels (in meters)
float distanceL = current_L_ticks * mt_per_tick;
float distanceR = current_R_ticks * mt_per_tick;

// 4. theta_dt calculation complies to positive CCW direction VERIFIED
// this approximation works (in radians) for small angles
float theta_dt = (distanceR-distanceL)/AXIS; // rad (dimensionless)
// linearizes the non-linear system around the current state
float meanDistance = (distanceL + distanceR) / 2.0;

// TO-DO: calculate velocities
// Vx = meanDistance / time_elapsed; // [mt/sec]
// Vth = theta_dt / time_elapsed; // [rad/sec]

// 5. Distance in x and y: (hypotenuse),cos,sen to calculate adjacent (x), opposite (y)
// assumes a heading towards +X at the WCS at the start
// forward means move towards positive X
// turn left means change heading towards positive Y, positive theta_r
float x_dt = meanDistance * cos(theta_dt); // complies with right-hand-rule
float y_dt = meanDistance * sin(theta_dt); // complies with right-hand-rule

// 6. Save the previous estimated pose for our next calculations
// Updated automatically at the beginning of the calculation cycle using est_pose()
float x_prev = *x_rPtr;
float y_prev = *y_rPtr;
float theta_prev = *theta_rPtr;
// 7.
// asign a value to whatever *x_rPtr, *y_rPtr and *theta_rPtr are pointing to
*x_rPtr = x_prev + x_dt; // in meters
*y_rPtr = y_prev + y_dt; // in meters

// 8. normalize heading, so that theta is always between 0 and 2 Pi
// above 2*Pi it wraps around and keeps increasing from 0
if((theta_prev + theta_dt) >= 6.28318)
    *theta_rPtr = (theta_prev + theta_dt) - 6.28318;
// below 0 it wraps around and start decreasing from 2*Pi on
else if((theta_prev + theta_dt) < 0)
    *theta_rPtr = (theta_prev + theta_dt) + 6.28318;
else
    *theta_rPtr = theta_prev + theta_dt; // rad (dimensionless)
}
