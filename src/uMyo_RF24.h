/*
  uMyo_RF24.h

  Copyright (c) 2022, Ultimate Robotics

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef UMYO_RF24_h
#define UMYO_RF24_h

#include <RF24.h>
#include "quat_math.h"
#define MAX_UMYO_DEVICES 12

typedef struct uMyo_data
{
	uint32_t id;
	int batt_mv;
	uint8_t last_data_id;
	int16_t cur_spectrum[4];
	int16_t raw_data[8];
	uint32_t last_data_time;
	sQ Qsg;
	operator=(uMyo_data d2)
	{
		id = d2.id;
		batt_mv = d2.batt_mv;
		last_data_id = d2.last_data_id;
		for(int x = 0; x < 4; x++) cur_spectrum[x] = d2.cur_spectrum[x];
		for(int x = 0; x < 8; x++) raw_data[x] = d2.raw_data[x];
		last_data_time = d2.last_data_time;
		Qsg = d2.Qsg;
	}
};

class uMyo_RF24_
{
private:
	RF24 *rf;
	uMyo_data devices[MAX_UMYO_DEVICES];
	sV nx, ny, nz;
	int device_count;
	uint8_t idToIdx(uint32_t id);
	uint8_t swapbits(uint8_t a);
	float asin_f(float x);
	float atan2_f(float y, float x);
public:
	uMyo_RF24_(void);
	void begin(int pin_cs, int pin_ce);
	void run();
	uint8_t getDeviceCount();
	int getBattery(uint8_t devidx);
	uint32_t getID(uint8_t devidx);
	uint8_t getDataID(uint8_t devidx);
	float getMuscleLevel(uint8_t devidx);
	void getSpectrum(uint8_t devidx, float *spectrum);
	void getRawData(uint8_t devidx, int16_t *data);
	float getPitch(uint8_t devidx);
	float getRoll(uint8_t devidx);
	float getYaw(uint8_t devidx);
};
extern uMyo_RF24_ uMyo;

#endif
