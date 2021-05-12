// James Graham 2021

#include <robot/connect/NodeConnect.h>
#include <robot/controller/PIDController.h>
#include <robot/motor/Motor.h>
#include <robot/SpeedPair.h>

using namespace robot;

// Our speed variables. Has setpoint and measurement floats.
SpeedPair leftPair;
SpeedPair rightPair;

// Left PIDController object.
robot::controller::PIDController leftPID{
	2,			// Kp gain
	10, 			// Ki gain
	0.02,			// Kd gain
	-255,			// Min PWM Value
	255,			// Max PWM Value
	100,			// Sample Period (ms)
	0,			// Tau Time Constant ()
	leftPair		// Current set point and measured value
};
// Right PIDController object.
robot::controller::PIDController rightPID{
	2,			// Kp gain
	10, 			// Ki gain
	0.02,			// Kd gain
	-255,			// Min PWM Value
	255,			// Max PWM Value
	100,			// Sample Period (ms)
	0,			// Tau Time Constant ()
	rightPair		// Current set point and measured value
};

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
	nodeHandle.init();
}

void loop()
{
	// Get Measurement Data.
	leftPair.measurement = leftMotor.retrieveMeasurement();
	rightPair.measurement = rightMotor.retrieveMeasurement();
	// Compute PID Values.
	leftPID.Compute();
	rightPID.Compute();
	// Set Motor PWMs. We have to cast the PID float output to an integer.
	leftMotor.setPWM(static_cast<int>(leftPID.getOutput()));
	rightMotor.setPWM(static_cast<int>(rightPID.getOutput()));
	// Update Node. This includes publishing.
	nodeHandle.update();
	// Wait 1ms.
	delay(1);
}
