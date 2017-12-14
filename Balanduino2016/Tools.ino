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

bool calibrateGyro()
{
    int16_t gyroXbuffer[25];
    for (uint8_t i = 0; i < 25; i++) {
	while (i2cRead(0x43, i2cBuffer, 2));
	gyroXbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
	delay(10);
    }
    if (!checkMinMax(gyroXbuffer, 25, 2000)) {
	Serial.println(F("Gyro calibration error"));
	buzzer::Set();
	return 1;
    }
    for (uint8_t i = 0; i < 25; i++)
	gyroXzero += gyroXbuffer[i];
    gyroXzero /= 25;
    return 0;
}

bool checkMinMax(int16_t * array, uint8_t length, int16_t maxDifference)
{				// Used to check that the robot is laying still while calibrating
    int16_t min = array[0], max = array[0];
    for (uint8_t i = 1; i < length; i++) {
	if (array[i] < min)
	    min = array[i];
	else if (array[i] > max)
	    max = array[i];
    }
    return max - min < maxDifference;
}
