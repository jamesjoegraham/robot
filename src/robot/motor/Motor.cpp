// James Graham 2021

#include "Motor.h"

#include <avr/interrupt.h>

using namespace robot::motor;

// ISR Global Variables. We keep them confined in this motor implementation.
volatile int right_temp_ticks;
volatile int left_temp_ticks;
volatile int32_t right_ticks = 0;
volatile int32_t left_ticks = 0;

//:: Interupt Service Routines (ISRs)
//
// These timers allow for accurate timing.
//

// Right Motor ISR
volatile unsigned int past_PINB = 0x00;
ISR(PCINT0_vect)
{
	// Did PINB0 just turn on? (past_PINB0 is 0, PINB0 is 1)
	if ((PINB & (1 << PB0)) && !(past_PINB & (1 << PB0))) // trigger on rising-edge only
	{
		// If PINB1 is on, decrement.
		if (PINB & (1 << PB1)) { right_temp_ticks--; right_ticks--; }  // reverse
		// Else increment.
		else { right_temp_ticks++; right_ticks++; }                // forward
	}
	past_PINB = PINB;
}

// Left Motor ISR
volatile unsigned int past_PIND = 0x00;
ISR(PCINT2_vect)
{
	// Did PIND0 just turn on? (past_PIND is 0, PIND0 is 1)
	if ((PIND & (1 << PD5)) && !(past_PIND & (1 << PD5))) // trigger on rising-edge only
	{
		// If PIND6 is on, decrement.
		if (PIND & (1 << PD6)) { left_temp_ticks--; left_ticks--; }  // reverse
		// Else increment.
		else { left_temp_ticks++; left_ticks++; }                // forward
	}
	past_PIND = PIND;
}

// Static ISR Result Floats.
volatile float omega_left_measured;
volatile float omega_right_measured;

// Measurement ISR
ISR(TIMER1_OVF_vect)
{
	// Constant used to convert from ticks to wheel RPM.
	constexpr float tick_to_rpm = 1.9073486328125;
	// Calculate the RPM from the ticks over a fixed time window
	// 16 MHz crystal / 8 = 2 MHz / 65536 counts = 30.52 Hz update rate
	// 8 encoder ticks x 120 gear = 960 ticks per revolution, 1 RPM = 16 ticks/sec
	// 2,000,000 / 1,048,576 = 1.9073... tick-to-rpm conv.
	omega_left_measured = (float)left_temp_ticks * tick_to_rpm;
	omega_right_measured = (float)right_temp_ticks * tick_to_rpm;
	
	// Reset temp tick counters for next measurement window     
	right_temp_ticks = 0;
	left_temp_ticks = 0;
}

// Initialize motor ISRs.
void Motor::init()
{
	// Set pin directions as inputs or outputs.
	DDRB |= (1 << PB3) | (1 << PB4); // PB3 = Output, Right PWM channel OC2A. PB4 = Output, Right direction.
	DDRD |= (1 << PD3) | (1 << PD4); // PD3 = Output, Left PWM channel OC2B. PD4 = Output, Left direction.
	DDRB &= ~((1 << PB0) | (1 << PB1)); // PB0 = Input, Right Encoder A ch.  PB1 = Input, Right Encoder B ch.
	DDRD &= ~((1 << PD5) | (1 << PD6)); // PD5 = Input, Left Encoder A ch.  PD6 = Input, Left Encoder B ch.

	// This is the actual timer register that counts. It is set at 0.
	TCNT2 = 0; // Timer2 start at zero.
	// Set the timer registers to PWM as desired.
	// Enable WGM21 and WGM20 (while WGM22 still off) for Fast PWM Mode.
	//
	// Fast PWM Mode starts the timer at 0 with the output turned on.
	// When the timers reasch the value  of compre register OCR2A/ORR2B
	// it turns off the output.
	//
	// Ex: OCR2A = 0 means 0% duty cycle.
	// Ex: OCR2A = 255 means 100% duty cycle.
	// Ex: OCR2A = 127 means 50% duty cycle.
	//
	TCCR2A = (1 << WGM21) | (1 << WGM20); // Timer2 Control Register A.
	// Clock mode (prescaling). It is currently set to clk/1. (16Mhz CPU).
	// It was previously set to CS21 (clk/8).
	TCCR2B = (1 << CS20); // Timer2 Control Register B.

	// These registers are the PWM level.
	OCR2A = 0; // Timer2 Output Compare Register A, 8-bit.
	OCR2B = 0; // Timer2 Output Compare Register B, 8-bit.

	// Set motor pin outputs to gnd or logic 0, 0 = off/forward, 1 = on/reverse
	PORTB &= ~((1 << PB3) | (1 << PB4));
	PORTD &= ~((1 << PD3) | (1 << PD4));

	// Set Timer 1 to monitor Encoders, interupt service routine uses
	TCNT1 = 0; // Timer1 start at zero
	TCCR1A = 0; // Timer1 Control Register A, enable normal mode.                                           
	TCCR1B = (1 << CS11); // Timer1 Control Register B.

	// Set Masks for Pin Change Interrupts.
	cli(); // Disable interrupts while changing registers.
	PCICR |= (1 << PCIE0) | (1 << PCIE2); // Pin Change Interrupt Control Register.
										  // Note: PB0 is PCINT0 on PCIE0, PD5 is PCINT21 on PCIE2.
	PCMSK0 = (1 << PCINT0); // Pin Change Mask Register, PB0 only interrupt.
	PCMSK2 = (1 << PCINT21); // Pin Change Mask Register, PD5 only interrupt.
	TIMSK1 = (1 << TOIE1); // Enable timer1 overflow interrupt (TOIE1).
	sei(); // Enable interrupts after config.       

}

// Retrieve the speed measurement.
float Motor::retrieveMeasurement() const
{
	// Return the static ISR result float.
	if (m_motorType == MotorType::left)
	{
		return omega_left_measured;
	}
	return omega_right_measured;
}

// Set the PWM of the motor.
void Motor::setPWM(int pwm)
{
	// Boolean that tells us if the motor is going forwards or not.
	const bool forwards{pwm > 0};
	// Find its absolute value.
	pwm = (pwm < 0) ? (-1 * pwm) : (pwm);
	// Clamp down the pwm to a max of 255.
	if (pwm > 255) { pwm = 255; }

	// If it's a left motor.
	if (m_motorType == MotorType::left)
	{
		// Use the prv_setLeftMotor helper function.
		prv_setLeftMotor(pwm, forwards);
	}
	// If it's a right motor.
	else
	{
		// Use the prv_setRightMotor helper function.
		prv_setRightMotor(pwm, forwards);
	}
}

// Helper Function. Set a left motor's PWM.
void Motor::prv_setLeftMotor(const uint8_t pwmValue, const bool forwards) const
{
	// If we're moving forwards.
	if (forwards)
	{
		// Set PORTD4 off.
		PORTD &= ~(1 << PD4);	
	}
	// If we're moving in reverse.
	else
	{
		// Set PORTD4 on.
		PORTD |= (1 << PD4);
	}

	// Set the motor PWM signal. If 0, turn off the motor.
	if (pwmValue == 0)
	{
		// Clear bits to disconnect OCR2B. 
		TCCR2A &= ~((1 << COM2B1) | (1 << COM2B0));
		// Reset value of timer OCR2B.
		OCR2B = 0;
		// Set output PORTD3 to 0.
		PORTD &= ~((1 << PD3));
	}
	// The motor is turned on.
	else
	{
		// Set bits to connect OCR2B to PIND6 and clear on compare match.
		TCCR2A |= (1 << COM2B1);
		// Set bits to connect OCR2B to PIND6 and clear on compare match.
		TCCR2A &= ~(1 << COM2B0);
		// Set OCR2B compare register as pwmValue.
		OCR2B = pwmValue;
	}

}

// Helper Function. Set a right motor's PWM.
void Motor::prv_setRightMotor(const uint8_t pwmValue, const bool forwards) const
{
	// If we're moving forwards.
	if (forwards)
	{
		// Set PORTB4 off.
		PORTB &= ~(1 << PB4);
	}
	// If we're moving in reverse.
	else
	{
		// Set PORTB4 on.
		PORTB |= (1 << PB4);
	}

	// Set the motor PWM signal. If 0, turn off the motor.
	if (pwmValue == 0)
	{
		// Clear bits to disconnect OCR2A.
		TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0));
		// Reset value of timer OCR2A.
		OCR2A = 0;
		// Set output PORTB3 to 0;
		PORTB &= ~((1 << PORTB3));
	}
	// The motor is turned on.
	else
	{
		// Set bits to connect OCR2A to PINB3 and clear on compare match.
		TCCR2A |= (1 << COM2A1);
		//Set bits to connect OCR2A to PINB3 and clear on compare match.
		TCCR2A &= ~(1 << COM2A0);
		// Set OCR2A compare register as pwmValue.
		OCR2A = pwmValue;
	}

}
