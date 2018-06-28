#define USE_USBCON // eliminates sync error of serial_node with a Teensy
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>

#define LEncoderA 2 // define encoder pins
#define LEncoderB 4
#define REncoderA 3
#define REncoderB 6

// For mapping admissible input values to admissible serial command values
float cmdMin = -0.3; // min addmisible input command value [mt/s]
float cmdMax = 0.3; // max addmisible input command value [mt/s]
float toLow = -63; // min addmisible serial value for Sabertooth [int]
float toHigh = 63; // max addmisible serial value for Sabertooth [int]

//////////////////////////////////////////////////////////////////////////////////////

float AXIS = 0.385; //distance between wheels in meters
float RADIUS = 0.0508; //wheel radius in meters (2 inch)
int TPR = 4096; //Encoder ticks per rotation 4x decoding
//sudo vars//////////////////////////////////////////////////////////////////////////
int OdomWait = 3; //first couple of times dont publish odom
int OdomCount = 0;
double WCS[2] = {0,0}; //global var WCS: write command speed

//ROS variables//////////////////////////////////////////////////////////////////////
/* ROS handler, instantiate with bigger buffer memory */
ros::NodeHandle_<ArduinoHardware, 25, 25, 512, 512> nh;
////ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("teensy_odom", &odom_msg);
geometry_msgs::Twist debug_msg;
ros::Publisher Debug ("debug", &debug_msg);

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
  WCS[0] = left_wheel_radians * RADIUS; //write new command speeds to global vars
  WCS[1] = right_wheel_radians * RADIUS;
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);
// Create two Encoder objects, using 2 pins for each:
Encoder LEncoder(LEncoderA, LEncoderB);
Encoder REncoder(REncoderA, REncoderB);

long EncoderVal[2] = {0,0};
double DDis[2] = {0,0}; //diferential of distance in meters
long Time[2] = {0,0};

//debug
double Vels[2] = {0,0}; // Wheel velocities [mt/s] from encoders

///program///////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial2.begin(9600); // initialize serial2 port, baud rate must match DIP switches
    nh.getHardware()->setBaud(57600); // default is 57600, also modify launch file of serial_node
    nh.initNode();
    nh.advertise(Pub);
    nh.advertise(Debug);
    nh.subscribe(subCmdVel);

    // nh.getParam("/serial_node/WheelSeparation", &WheelSeparation,1);
    // nh.getParam("/serial_node/WheelDiameter", &WheelDiameter,1);
    // nh.getParam("/serial_node/IMax", &IMax,1);
    // nh.getParam("/serial_node/AccParam", &AccParam,1);


}

void loop(){

    nh.spinOnce();

    //first couple of times dont publish odom
    if(OdomCount > OdomWait){
	    odom_msg.linear.x = Vels[0]; // Left Wheel (TODO:change message type)
	    odom_msg.linear.y = Vels[1]; // Right Wheel (TODO:change message type)
		Pub.publish(&odom_msg);
	}
	else{OdomCount++;}

    debug_msg.linear.x = WCS[0]; // Write command speed [mt/s]
    debug_msg.linear.y = Vels[0]; // Wheel velocities [mt/s] from encoders
    debug_msg.angular.x = WCS[1]; // Write command speed [mt/s]
    debug_msg.angular.y = Vels[1]; // Wheel velocities [mt/s] from encoders


    Debug.publish(&debug_msg);

    // //safeswitch
    // if(CheckBumpers()){
    // 	WCS[0]=0;
    // 	WCS[1]=0;
    // }

    MotorWrite(); // send cmd to motors, read encoder values

    delay(100);
}


// int CheckBumpers(){
// 	int ToReturn = 0;
// 	for(int i = 0;i<8;i++){
// 		if(digitalRead(Button[i])) ToReturn = 1;
// 	}
// 	return ToReturn;
// }

//encoder code//////////////////////////////////////////////////////////

//motor write speed - in motor units
double MWS[2]= {0,0};

double EncoderSpeed(int M){
	//if fist time in program return 0 and init time vars
	if(Time[0]==0 && Time[1] == 0){
		Time[0] = millis();
		Time[1] = millis();
		return 0;
	}

	//read encoder ticks
	if(M == 0){
		EncoderVal[0] = LEncoder.read();
		LEncoder.write(0); // reset encoder to zero
	}
	if(M == 1){
		EncoderVal[1] = -1*REncoder.read(); // sign change (R-motor turns CW, R-encoder CCW when robot moves forward)
		REncoder.write(0); // reset encoder to zero
	}

	//differencial of time in seconds
	long T = millis();
	int DTime = T-Time[M];
	Time[M] = T;


	//diferential of distance in meters
	DDis[M] = TicksToMeters(EncoderVal[M]);

	//calculate short term measured velocity
	double EVel = (DDis[M]/DTime)*1000;

	// return encoder speed [mt/s] to publish to /teensy_odom [mt/s]
    return EVel;

}

double TicksToMeters(int Ticks){
	return (Ticks*3.14*RADIUS*2)/TPR;
}

//motor codes///////////////////////////////////////////////////////////
void MotorWrite(){

	for(int i = 0; i<2; i++){

		//encoder data
		Vels[i] = EncoderSpeed(i);

	}
    moveLeftMotor(WCS[0]);  // move left motor
    moveRightMotor(WCS[1]);  // move right motor

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
