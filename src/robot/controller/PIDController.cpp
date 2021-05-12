// James Graham 2021

#include "PIDController.h"

using namespace robot::controller;

namespace
{
	// Helper function that clamps down value to a min and max.
	void clamp(float& value, const float low, const float high)
	{
		if (value < low)
		{
			value = low;
		}
		else if (value > high)
		{
			value = high;
		}
	}
}

// PIDController Constructor.
PIDController::PIDController(
	float Kp,
	float Ki,
	float Kd,
	float min,
	float max,
	float dt,
	float tau,
	SpeedPair& sp
	) 
	: 
	m_Kp{Kp},
	m_Ki{Ki},
	m_Kd{Kd},
	m_min{min},
	m_max{max},
	m_dt{dt},
	m_tau{tau},
	m_speedPair{sp}
{
	m_prevError = 0.0f;
	m_prevMeasurement = 0.0f;
	m_integrator = 0.0f;
	m_differentiator = 0.0f;
	m_output = 0.0f;
}

// This is where the primary calculations happen.
void PIDController::Compute()
{
	// The final output value. It will be the sum of a:
	//	- Proprotional Term (P)
	//	- Integrator Term (I)
	//	- Differentiator Term (D)
	float result = 0;

	// References to the set point and measurement values.
	const float& setPoint = m_speedPair.setpoint;
	const float& measurement = m_speedPair.measurement;

	// The error term is how far away we are from the reference term. Since we're working in speed,
	// you can think of this value as having units m/s.
	const float error = setPoint - measurement;

	//:: Proportional Term
	//
	// This term is simply the product of the Kp gain and the error value.
	//
	const float prop = m_Kp * error;
	// Add it to the total.
	result += prop;

	//:: Integrator Term
	//
	// This term is a product of the running total sum of errors.
	//
	// Note that m_Ki * m_dt * 0.5f is the height of the Riemann rectangle 
	// and (error + m_prevError) is its height. We're using time as our
	// independent variable.
	//
	//					   WIDTH				  HEIGHT
	m_integrator += (m_Ki * m_dt * 0.5f) * (error + m_prevError);
	// Dyanmic integrator clamping for anti-wind up. Remember that result is still the proportional value.
	// This clamps based on how far or below the proportional value prop.
	clamp(
		m_integrator,
		(m_min < prop) ? m_min - prop : 0.0f,
		(m_max > prop) ? m_max - prop : 0.0f);
	// Add it to the total.
	result += m_integrator;

	//:: Differentiator Term
	//
	// This calculation also includes a low pass filter. These high frequencies may cause a high pitch to play in the motors.
	//
	m_differentiator =
		(2.0f * -m_Kd * (measurement - m_prevMeasurement))
		+ ((2.0f * m_tau - m_dt) * m_differentiator)
		/ (2.0f * m_tau + m_dt);
	// Add it to the total.
	result += m_differentiator;

	// Clamp the final value to our limits.
	clamp(result, m_min, m_max);

	// Set the previous values as the current.
	m_prevError = error;
	m_prevMeasurement = measurement;

	// Set the m_output as result.
	m_output = result;
}
