#ifndef Motor_HEADER
#define Motor_HEADER

#include <stdint.h>

namespace robot { namespace motor
{
	// MotorType object specifying which side of the robot the motor is on.
	enum class MotorType {left, right};
	
	// Motor
	//
	// The Motor object handles all instances of motors. This includes PWM adjustment with setPWM() and speed
	// measurement reading with retrieveMeasurement(). This object handles all of the timers needed to run the PWM
	// signal and record the speed measurements.
	class Motor
	{
	public:
		// Constructor

		// Constructor takes a MotorType.
		Motor(const MotorType type) : m_motorType{type}
		{};

		// Initialize motor funcitons.
		static void init();

		// Set the PWM value.
		void setPWM(int pwm);

		// Retrieve the speed measurement. Returns the measurement value.
		float retrieveMeasurement() const;

		int32_t getTicks() const;


	private:
		// Helper functions

		// Change the PWM setting and direction value of the left motor.
		void prv_setLeftMotor(const uint8_t pwmValue, const bool forwards) const;

		// Change the PWM setting and direction value of the right motor.
		void prv_setRightMotor(const uint8_t pwmValue, const bool forwards) const;


	private:
		// What type of motor this is.
		const MotorType m_motorType;

	};
}}

#endif // !Motor_HEADER
