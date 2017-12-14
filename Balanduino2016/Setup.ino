

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

 This is the algorithm for the Balanduino balancing robot.
 It can be controlled by either an Android app or a computer application via Bluetooth.
 The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 A dedicated Windows application can be found here: https://github.com/TKJElectronics/BalanduinoWindowsApp
 It can also be controlled by a PS3, PS4, Wii or a Xbox controller.
 Furthermore it supports the Spektrum serial protocol used for RC receivers.
 For details, see: http://balanduino.net/
*/

#include "Balanduino.h"
#include <Arduino.h>		// Standard Arduino header
#include <Wire.h>		// Official Arduino Wire library

void initialize()
{
    /* Setup buzzer pin */
    buzzer::SetDirWrite();

    /* Read the PID values, target angle and other saved values in the EEPROM */
    if (!checkInitializationFlags()) {
	readEEPROMValues();	// Only read the EEPROM values if they have not been restored
    } else {			// Indicate that the EEPROM values have been reset by turning on the buzzer
	buzzer::Set();
	delay(1000);
	buzzer::Clear();
	delay(100);		// Wait a little after the pin is cleared
    }

    /* Initialize UART */
    Serial.begin(115200);

    /* Setup encoders */
    leftEncoder1::SetDirRead();
    leftEncoder2::SetDirRead();
    rightEncoder1::SetDirRead();
    rightEncoder2::SetDirRead();
    leftEncoder1::Set();	// Enable pull-ups
    leftEncoder2::Set();
    rightEncoder1::Set();
    rightEncoder2::Set();

#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
    /* Enable encoder pins interrupt sources */
    *digitalPinToPCMSK(leftEncoder1Pin) |=
	(1 << digitalPinToPCMSKbit(leftEncoder1Pin));
    *digitalPinToPCMSK(rightEncoder1Pin) |=
	(1 << digitalPinToPCMSKbit(rightEncoder1Pin));
    *digitalPinToPCMSK(leftEncoder2Pin) |=
	(1 << digitalPinToPCMSKbit(leftEncoder2Pin));
    *digitalPinToPCMSK(rightEncoder2Pin) |=
	(1 << digitalPinToPCMSKbit(rightEncoder2Pin));

    /* Enable pin change interrupts */
    *digitalPinToPCICR(leftEncoder1Pin) |=
	(1 << digitalPinToPCICRbit(leftEncoder1Pin));
    *digitalPinToPCICR(rightEncoder1Pin) |=
	(1 << digitalPinToPCICRbit(rightEncoder1Pin));
    *digitalPinToPCICR(leftEncoder2Pin) |=
	(1 << digitalPinToPCICRbit(leftEncoder2Pin));
    *digitalPinToPCICR(rightEncoder2Pin) |=
	(1 << digitalPinToPCICRbit(rightEncoder2Pin));
#else
#error "Please define "PIN_CHANGE_INTERRUPT_VECTOR_LEFT" and "PIN_CHANGE_INTERRUPT_VECTOR_RIGHT" in Balanduino.h"
#endif

    /* Set the motordriver diagnostic pins to inputs */
    leftDiag::SetDirRead();
    rightDiag::SetDirRead();

    /* Setup motor pins to output */
    leftPWM::SetDirWrite();
    leftA::SetDirWrite();
    leftB::SetDirWrite();
    rightPWM::SetDirWrite();
    rightA::SetDirWrite();
    rightB::SetDirWrite();

    /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf page 129-139 */
    // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
    TCCR1B = (1 << WGM13) | (1 << CS10);	// Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
    ICR1 = PWMVALUE;		// ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

    /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
    // Clear OC1A/OC1B on compare match when up-counting
    // Set OC1A/OC1B on compare match when down-counting
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);

    /* Attach onInit function */
    // This is used to set the LEDs according to the voltage level and vibrate the controller to indicate the new connection

    /* Setup IMU */
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2;	// Set I2C frequency to 400kHz

    while (i2cRead(0x75, i2cBuffer, 1));
    if (i2cBuffer[0] != 0x68) {	// Read "WHO_AM_I" register
	Serial.print(F("Error reading sensor"));
	buzzer::Set();
	while (1);		// Halt
    }

    while (i2cWrite(0x6B, 0x80, true));	// Reset device, this resets all internal registers to their default values
    do {
	while (i2cRead(0x6B, i2cBuffer, 1));
    } while (i2cBuffer[0] & 0x80);	// Wait for the bit to clear
    delay(5);
    while (i2cWrite(0x6B, 0x09, true));	// PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
#if 1
    i2cBuffer[0] = 1;		// Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
    i2cBuffer[1] = 0x03;	// Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
#else
    i2cBuffer[0] = 15;		// Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
    i2cBuffer[1] = 0x00;	// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
#endif
    i2cBuffer[2] = 0x00;	// Set Gyro Full Scale Range to ±250deg/s
    i2cBuffer[3] = 0x00;	// Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cBuffer, 4, true));	// Write to all four registers at once

    delay(100);			// Wait for the sensor to get ready

    /* Set Kalman and gyro starting angle */
    while (i2cRead(0x3D, i2cBuffer, 4));
    int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // We then convert it to 0 to 2π and then from radians to degrees
    double accAngle =
	(atan2((double) accY - cfg.accYzero, (double) accZ - cfg.accZzero)
	 + PI) * RAD_TO_DEG;

    kalman.setAngle(accAngle);	// Set starting angle
    pitch = accAngle;
    gyroAngle = accAngle;

    /* Calibrate gyro zero value */
    while (calibrateGyro());	// Run again if the robot is moved while calibrating

    LED::SetDirWrite();		// Set LED pin to output
    stopAndReset();		// Turn off motors and reset different values

    /* Beep to indicate that it is now ready */
    buzzer::Set();
    delay(100);
    buzzer::Clear();

    /* Setup timing */
    kalmanTimer = micros();
    pidTimer_us = kalmanTimer;
    imuTimer = millis();
    encoderTimer = imuTimer;
    reportTimer = imuTimer;
    ledTimer = imuTimer;
    blinkTimer = imuTimer;
}
