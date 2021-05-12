// James Graham 2021

#include <robot/connect/NodeConnect.h>

// TODO: implement our custom PID Controller.
//#include <robot/controller/PIDController.h>
#include <robot/motor/Motor.h>
#include <robot/SpeedPair.h>

#include <PID_v1.h>

// DEBUG Mode specified in make
#ifndef DEBUG
#define DEBUG 0
#endif

using namespace robot;

// TODO: put in anonymous namespace
// Our speed variables. Has setpoint and measurement floats.
SpeedPair leftPair;
SpeedPair rightPair;

// Left PIDController object.
/*robot::controller::PIDController leftPID{
	2,				// Kp gain
	10, 			// Ki gain
	0.02,			// Kd gain
	-255,			// Min PWM Value
	255,			// Max PWM Value
	100,			// Sample Period (ms)
	0.02,			// Tau Time Constant ()
	leftPair		// Current set point and measured value
};
// Right PIDController object.
robot::controller::PIDController rightPID{
	2,				// Kp gain
	10, 			// Ki gain
	0.02,			// Kd gain
	-255,			// Min PWM Value
	255,			// Max PWM Value
	100,			// Sample Period (ms)
	0.02,			// Tau Time Constant ()
	rightPair		// Current set point and measured value
};*/

double omega_right_measured1 = 0.0f;
double omega_right_measured_abs = 0.0f;

double omega_right_setpoint = 0.0f;
double omega_right_setpoint_abs = 0.0f;
double right_pwm = 0.0f;

double omega_left_measured1;
double omega_left_measured_abs = 0.0f;

double omega_left_setpoint = 0.0f;
double omega_left_setpoint_abs = 0.0f;
double left_pwm = 0.0f;

PID rightPID(
	&omega_right_measured_abs,
	&right_pwm,
	&omega_right_setpoint_abs,
	2,
	10,
	0.02,
	DIRECT
);
PID leftPID(
	&omega_left_measured_abs,
	&left_pwm,
	&omega_left_setpoint_abs,
	2,
	10,
	0.02,
	DIRECT
);

// Declare our motor objects.
motor::Motor leftMotor{robot::motor::MotorType::left};
motor::Motor rightMotor{robot::motor::MotorType::right};	

// Our ROS Node Handle Object
connect::NodeConnect nodeHandle{leftPair, rightPair};

// Entry point.
void setup()
{
	// Initialize only once.
	leftMotor.init();
	
	// DEBUG Mode for manual Serial Monitor output.
	if (DEBUG) { Serial.begin(9600); }
	else { nodeHandle.init(); }

	rightPID.SetMode(AUTOMATIC);
	rightPID.SetSampleTime(100); // 100ms
	leftPID.SetMode(AUTOMATIC);
	leftPID.SetSampleTime(100); // 100ms
}

// Continuous loop.
void loop()
{
	// Get Measurement Data.
	//leftPair.measurement = leftMotor.retrieveMeasurement();
	//rightPair.measurement = rightMotor.retrieveMeasurement();
	omega_right_measured1 = (double)rightMotor.retrieveMeasurement();
	omega_right_measured_abs = abs(omega_right_measured1);
	omega_left_measured1 = (double)leftMotor.retrieveMeasurement();
	omega_left_measured_abs = abs(omega_left_measured1);

	omega_left_setpoint = (double)leftPair.setpoint;
	omega_left_setpoint_abs = abs(omega_left_setpoint);
	omega_right_setpoint = (double)rightPair.setpoint;
	omega_right_setpoint_abs = abs(omega_right_setpoint);

	// Compute PID Values.
	const bool leftComputed = leftPID.Compute();
	const bool rightComputed = rightPID.Compute();
	// Set Motor PWMs. We have to cast the PID float output to an integer.
	if (leftComputed)
	{
		//leftMotor.setPWM(static_cast<int>(leftPID.getOutput() * (leftPair.setpoint < 0) ? -1 : 1));
		leftMotor.setPWM((int)left_pwm * ((omega_left_setpoint < 0) ? -1 : 1));
	}
	if (rightComputed)
	{
		//rightMotor.setPWM(static_cast<int>(rightPID.getOutput() * (rightPair.setpoint < 0) ? -1 : 1));
		rightMotor.setPWM((int)right_pwm * ((omega_right_setpoint < 0) ? -1 : 1));
	}

	// DEBUG Info
	if (DEBUG)
	{
		Serial.print("Left Motor: S = ");
		Serial.print(leftPair.setpoint);
		Serial.print(" M = ");
		Serial.print(leftPair.measurement);
		Serial.print("Right Motor: S = ");
		Serial.print(rightPair.setpoint);
		Serial.print(" M = ");
		Serial.print(rightPair.measurement);
		Serial.print("\n");
	}
	else
	{
		// Update Node. This includes publishing.
		nodeHandle.update();
	}

	// Wait 1ms.
	delay(1);
}
