// James Graham 2021

#include "NodeConnect.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

using namespace robot::connect;

// Calculation Constants
constexpr float wheel_radius = 0.0325; // (m).
constexpr float wheel_axis = 0.200; // (m).
constexpr float pi = 3.14159265359;

float leftSetPoint;
float rightSetPoint;

void subscriberCallbackVel(const geometry_msgs::Twist& msg)
{
	leftSetPoint = (float)(((msg.linear.x / wheel_radius) - ((msg.angular.z * wheel_axis) / (2 * wheel_radius))) * 30 / pi);
	rightSetPoint = (float)(((msg.linear.x / wheel_radius) + ((msg.angular.z * wheel_axis) / (2 * wheel_radius))) * 30 / pi);
};
ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", &subscriberCallbackVel);

bool fireLED = false;
void subscriberCallbackLED(const std_msgs::Bool& msg)
{
	fireLED = msg.data;
}
ros::Subscriber<std_msgs::Bool> LEDSub("fireLED", &subscriberCallbackLED);

NodeConnect::NodeConnect(SpeedPair& leftPair, SpeedPair& rightPair)
    :
    m_leftPair(leftPair),
    m_rightPair(rightPair),
    m_leftTicks("lwheel", &m_leftMsg),
    m_rightTicks("rwheel", &m_rightMsg)
{}

void NodeConnect::init()
{
	m_nodeHandle.initNode();
	m_nodeHandle.subscribe(velSub);
	m_nodeHandle.subscribe(LEDSub);
	m_nodeHandle.advertise(m_leftTicks);
	m_nodeHandle.advertise(m_rightTicks);
}

void NodeConnect::refresh(int32_t leftTicks, int32_t rightTicks)
{
	// Covnert Measured RPM values back to TICK
	constexpr float tick_to_rpm = 1.9073486328125;
	// Update left values.
	m_leftPair.setpoint = leftSetPoint;
	// Set the ticks.
	m_leftMsg.data = leftTicks;
	// Update right values.
	m_rightPair.setpoint = rightSetPoint;
	// Set the ticks.
	m_rightMsg.data = rightTicks;
}

bool NodeConnect::getFireLED()
{
	return fireLED;
}

void NodeConnect::update()
{
	// Update node handle
	m_nodeHandle.spinOnce();
}
