// James Graham 2021

#include "NodeConnect.h"

#include <geometry_msgs/Twist.h>

using namespace robot::connect;

// Calculation Constants
constexpr float wheel_radius = 0.0325; // (m).
constexpr float wheel_axis = 0.200; // (m).
constexpr float tick_to_rpm = 1.9073486328125;
constexpr float pi = 3.14159265359;

float leftSetPoint;
float rightSetPoint;

void subscriberCallback(const geometry_msgs::Twist& msg)
{
	leftSetPoint = ((msg.linear.x / wheel_radius) + ((msg.angular.z * wheel_axis) / (2 * wheel_radius))) * 30 / pi;
	rightSetPoint = ((msg.linear.x / wheel_radius) - ((msg.angular.z * wheel_axis) / (2 * wheel_radius))) * 30 / pi;
};
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &subscriberCallback);

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
	m_nodeHandle.subscribe(sub);
	m_nodeHandle.advertise(m_leftTicks);
	m_nodeHandle.advertise(m_rightTicks);
}

void NodeConnect::update()
{
	// Update left values.
	m_leftPair.setpoint = leftSetPoint;
	m_leftMsg.data = (int32_t)m_leftPair.measurement;
	// Update right values.
	m_rightPair.setpoint = rightSetPoint;
	m_rightMsg.data = (int32_t)m_rightPair.measurement;
	// Update publishers
	m_leftTicks.publish(&m_leftMsg);
	m_rightTicks.publish(&m_rightMsg);
	// Update node handle
	m_nodeHandle.spinOnce();
}
