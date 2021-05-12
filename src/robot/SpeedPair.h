#ifndef SpeedPair_HEADER
#define SpeedPair_HEADER

namespace robot
{
	// Pair of set point and measurement data structure.
	struct SpeedPair
	{
		// Constructor
		SpeedPair(float sp = 0.0f, float m = 0.0f)
			: setpoint(sp), measurement(m)
		{}

		float setpoint;
		float measurement;
	};
}


#endif // !SpeedPair_Header
