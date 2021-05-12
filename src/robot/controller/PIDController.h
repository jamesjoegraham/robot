// James Graham 2021

#ifndef PIDController_HEADER
#define PIDController_HEADER

#include <robot/SpeedPair.h>

namespace robot { namespace controller
{
	// Instance that controls the calculations for a PID controller.
	//
	// This PID Controller includes a low pass filtering for high frequency derivative noise and 
	// dynamic integrator clamping.
	//
	class PIDController
	{
	public:
		// Constructor

		// PIDController takes its options and the set point and measurement locations.
		//
		//	Options:
		//
		//	Kp: Proportional gain
		//	Ki: Integrator gain
		//	Kd: Differentiator gain
		//	min: min value of output
		//	max: max value of output
		//	samplePeriod: Sampling Period (ms)
		//	tau: Differentiator time constant.
		//	sp: SpeedPair reference.
		//
		PIDController(
			float Kp,
			float Ki,
			float Kd,
			float min,
			float max,
			float samplePeriod,
			float tau,
			SpeedPair& sp
			);

		// Execute the calculations.
		void Compute();

		// Output getter. This returns by const reference to make it read only.
		float getOutput() const
		{
			return m_output;
		}

	private:
		
		// The proportional tuning constant.
		const float m_Kp;

		// The integral tuning constant.
		const float m_Ki;

		// The derivative tuning constant.
		const float m_Kd;

		// The minimum allowed value.
		const float m_min;

		// The maximum allowed value. PIDController::Compute() clamps the output down to this value as a maximum.
		const float m_max;

		// Our dt, or also known as change in time. This number represents how long each tick of the controller is.
		const float m_dt;

		// Our derivative low-pass filter time constant.
		const float m_tau;

		// The previous error from the last computation.
		float m_prevError;

		// The previous measurement from the last computation.
		float m_prevMeasurement;

		// The summed integrator value.
		float m_integrator;

		// The differentiator value.
		float m_differentiator;

		// This object stores the set point (first) and measured value (second). The reference allows for
		// zero-copying use throughout the program.
		SpeedPair& m_speedPair;

		// PIDController's output.
		float m_output;

	};

}}


#endif // !PIDController_HEADER
