/* Copyright (C) 2013-2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

void moveMotor(Command motor, Command direction, double speedRaw)
{				// Speed is a value in percentage 0-100%
    if (speedRaw > 100)
	speedRaw = 100.0;
    setPWM(motor, speedRaw * ((double) PWMVALUE) / 100.0);	// Scale from 0-100 to 0-PWMVALUE
    if (motor == left) {
	if (direction == forward) {
	    leftA::Clear();
	    leftB::Set();
	} else {
	    leftA::Set();
	    leftB::Clear();
	}
    } else {
	if (direction == forward) {
	    rightA::Set();
	    rightB::Clear();
	} else {
	    rightA::Clear();
	    rightB::Set();
	}
    }
}

void stopMotor(Command motor)
{
    setPWM(motor, PWMVALUE);	// Set high
    if (motor == left) {
	leftA::Set();
	leftB::Set();
    } else {
	rightA::Set();
	rightB::Set();
    }
}

void setPWM(Command motor, uint16_t dutyCycle)
{				// dutyCycle is a value between 0-ICR1
    if (motor == left)
	OCR1A = dutyCycle;
    else
	OCR1B = dutyCycle;
}

void stopAndReset()
{
    stopMotor(left);
    stopMotor(right);
    lastError = 0;
    integratedError = 0;
    targetPosition = getWheelsPosition();
    lastRestAngle = cfg.targetAngle;
}

/* Interrupt routine and encoder read functions */
// It uses gray code to detect if any pulses are missed. See: https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino and http://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder.

#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
static const int8_t enc_states[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };	// Encoder lookup table if it interrupts on every edge

ISR(PIN_CHANGE_INTERRUPT_VECTOR_LEFT)
{
    leftEncoder();
}

ISR(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
{
    rightEncoder();
}
#endif

void leftEncoder()
{
    static uint8_t old_AB = 0;
    old_AB <<= 2;		// Remember previous state
    old_AB |=
	(leftEncoder2::
	 IsSet() >> (leftEncoder2::Number -
		     1)) | (leftEncoder1::IsSet() >> leftEncoder1::Number);
    leftCounter -= enc_states[old_AB & 0x0F];
}

void rightEncoder()
{
    static uint8_t old_AB = 0;
    old_AB <<= 2;		// Remember previous state
    old_AB |=
	(rightEncoder2::
	 IsSet() >> (rightEncoder2::Number -
		     1)) | (rightEncoder1::IsSet() >> rightEncoder1::
			    Number);
    rightCounter += enc_states[old_AB & 0x0F];
}

int32_t readLeftEncoder()
{				// The encoders decrease when motors are traveling forward and increase when traveling backward
    return leftCounter;
}

int32_t readRightEncoder()
{
    return rightCounter;
}

int32_t getWheelsPosition()
{
    return leftCounter + rightCounter;
}
