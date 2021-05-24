#ifndef NodeConnect_HEADER
#define NodeConnect_HEADER

#include <robot/SpeedPair.h>

#include <ros.h>
#include <std_msgs/Int32.h>

namespace robot { namespace connect
{
	// ROS Handling Class
	class NodeConnect
	{
	public:

		// Constructor. We specify where we want our value set point values to go here.
		NodeConnect(SpeedPair& leftPair, SpeedPair& rightPair);

		// Initialize node.
		void init();

		// Refresh publisher readings.
		void refresh(int32_t leftTicks, int32_t rightTicks);

		// Update set points and measurements.
		void update();

		ros::NodeHandle& getHandle()
		{ return m_nodeHandle; }

	private:

		// Node handle.
		ros::NodeHandle m_nodeHandle;

		// Locations of measurement and set points.
		SpeedPair& m_leftPair;
		SpeedPair& m_rightPair;
		
	public:
		std_msgs::Int32 m_leftMsg;
		std_msgs::Int32 m_rightMsg;

		// Publisher handles
		ros::Publisher m_leftTicks;
		ros::Publisher m_rightTicks;

	};
}}

#endif // !NodeConnect_HEADER
