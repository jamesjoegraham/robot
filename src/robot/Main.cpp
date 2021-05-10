// James Graham 2021

#include <ros.h>

#include "Motor.h"
#include "NodeConnect.h"
#include "PIDController.h"

using namespace robot;
using namespace robot::connect;

int main()
{
	// Our speed variables for the left motor.
	SpeedPair leftPair;

	// PIDController object.
	robot::controller::PIDController leftPID{
		// Kp gain
		2,
		// Ki gain
		10,
		// Kd gain
		0.02,
		// Min PWM Value
		-255,
		// Max PWM Value
		255,
		// Sample Period (ms)
		100,
		// Tau Time Constant ()
		0.5,
		// Current set point and measured value
		leftPair};

	// Our speed variables for the right motor.
	SpeedPair rightPair;

	// PIDController object.
	robot::controller::PIDController rightPID{
		2,
		10,
		0.02,
		-255,
		255,
		100,
		0.5,
		rightPair};

	// Declare our motor objects.
	motor::Motor leftMotor{robot::motor::MotorType::left};
	motor::Motor rightMotor{robot::motor::MotorType::right};

	// Initialize only once.
	leftMotor.init();

	// Initialize ROS Node connection
	connect::NodeConnect nodeHandle{leftPair, rightPair};

	// The program's main loop.
	while (1)
	{
		// Get Measurement Data.
		leftPair.measurement = leftMotor.retrieveMeasurement();
		rightPair.measurement = rightMotor.retrieveMeasurement();
		// Compute PID Values.
		leftPID.Compute();
		rightPID.Compute();
		// Set Motor PWMs. We have to cast the PID float output to the int PWM.
		leftMotor.setPWM(static_cast<int>(leftPID.getOutput()));
		rightMotor.setPWM(static_cast<int>(rightPID.getOutput()));
		// Update Node
		nodeHandle.update();
		// Wait 1ms
		delay(1);
	}

	return 0;
}
