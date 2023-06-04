/*
  uMyo_RF24.cpp

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
#include <SPI.h>
#include <RF24.h>
#include "uMyo_RF24.h"

uMyo_RF24_::uMyo_RF24_(void)
{
	device_count = 0;
	nx.x = 1; ny.x = 0; nz.x = 0; //for angles calculation
	nx.y = 0; ny.y = 1; nz.y = 0;
	nx.z = 0; ny.z = 0; nz.z = 1;
}

uint8_t uMyo_RF24_::swapbits(uint8_t a)
{ //uMyo pipe address uses swapped bits order
  // reverse the bit order in a single byte
    uint8_t v = 0;
    if(a & 0x80) v |= 0x01;
    if(a & 0x40) v |= 0x02;
    if(a & 0x20) v |= 0x04;
    if(a & 0x10) v |= 0x08;
    if(a & 0x08) v |= 0x10;
    if(a & 0x04) v |= 0x20;
    if(a & 0x02) v |= 0x40;
    if(a & 0x01) v |= 0x80;
    return v;
}

void uMyo_RF24_::begin(int pin_cs, int pin_ce)
{
	rf = new RF24(pin_ce, pin_cs, 1000000);
	uint8_t pipe_rx[8] = {0x0E, 0xE6, 0x0D, 0xA7, 0, 0, 0, 0};
	for(int x = 0; x < 8; x++) //nRF24 and uMyo have different bit order for pipe address
		pipe_rx[x] = swapbits(pipe_rx[x]);

	rf->begin();
	rf->setDataRate(RF24_1MBPS);
	rf->setAddressWidth(4);
	rf->setChannel(83);
	rf->setRetries(0, 0);
	rf->setAutoAck(0);
	rf->disableDynamicPayloads();
	rf->setPayloadSize(32);
	rf->openReadingPipe(0, pipe_rx);
	rf->setCRCLength(RF24_CRC_DISABLED);
	rf->disableCRC();
//	rf->printDetails();
	rf->startListening(); //listen for uMyo data
	rf->printDetails(); 
//	Serial.println("uMyo lib init");
	protocol_version = 3; //set as default for newer devices
}

uint8_t uMyo_RF24_::idToIdx(uint32_t id)
{
	uint32_t ms = millis();
	for(uint8_t u = 0; u < device_count; u++)
	{
		if(ms - devices[u].last_data_time > 5000)
		{
			for(int u1 = u+1; u1 < device_count; u1++)
			{
				devices[u1-1] = devices[u1];
			}
			device_count--;
		}
	}
	for(uint8_t u = 0; u < device_count; u++)
		if(id == devices[u].id)
		{
			devices[u].last_data_time = ms;
			return u;
		}
	if(device_count < MAX_UMYO_DEVICES)
	{
		uint8_t u = device_count;
		devices[u].id = id;
		devices[u].last_data_time = ms;
		device_count++;
	}
	return device_count-1;
}

void uMyo_RF24_::run()
{
	if(!rf->available()) return;
	uint8_t rf_pack[33];
	rf->read(rf_pack, 33); //processing packet
	uint8_t *in_pack = rf_pack+1; //ignore 1st byte

	byte data_id = in_pack[0];
	byte message_length = in_pack[1];
	byte chk1 = in_pack[30];
	byte chk2 = in_pack[31];
	byte sum1 = 0, sum2 = 0;
	for(int x = 0; x < 30; x++)
	{
		sum1 += in_pack[x];
		sum2 += sum1;
	}
	uint8_t check_ok = 1;
	if(sum1 != chk1)// || sum2 != chk2)
		check_ok = 0;

	if(!check_ok)
	{
		return;
	}
	byte u1 = in_pack[2];//32-bit unit ID, unique for every uECG device
	byte u2 = in_pack[3];
	byte u3 = in_pack[4];
	byte u4 = in_pack[5];
	uint32_t id = (u1<<24) | (u2<<16) | (u3<<8) | u4;
	uint8_t u = idToIdx(id);
	devices[u].last_data_id = data_id;

	uint8_t ppos = 6;
	devices[u].batt_mv = 2000 + 10 * in_pack[ppos++];
	int muscle_avg = 0;
	for(uint8_t x = 0; x < 4; x++)
	{
		if(x == 0)
		{
			devices[u].cur_spectrum[x] = in_pack[ppos]<<8;
			uint16_t muscle_avg = in_pack[ppos+1];
			devices[u].device_avg_muscle_level = (muscle_avg*muscle_avg)>>3;
		}
		else
			devices[u].cur_spectrum[x] = (in_pack[ppos]<<8)|in_pack[ppos+1];
		ppos += 2;
	}
	int16_t w, x, y, z;
	w = (in_pack[ppos]<<8)|in_pack[ppos+1]; ppos += 2;
	x = in_pack[ppos++]<<8;
	y = in_pack[ppos++]<<8;
	z = in_pack[ppos++]<<8;
	devices[u].Qsg.w = 0.00003125*(float)w;
	devices[u].Qsg.x = 0.00003125*(float)x;
	devices[u].Qsg.y = 0.00003125*(float)y;
	devices[u].Qsg.z = 0.00003125*(float)z;
	q_renorm(&devices[u].Qsg);

	devices[u].raw_data[0] = (in_pack[ppos]<<8)|in_pack[ppos+1]; ppos += 2;
	uint8_t sign_byte = in_pack[ppos+7];
	for(uint8_t x = 0; x < 7; x++)
	{
		if(sign_byte & 1<<(6-x)) devices[u].raw_data[1+x] = devices[u].raw_data[0] - in_pack[ppos++];
		else devices[u].raw_data[1+x] = devices[u].raw_data[0] + in_pack[ppos++];
	}
}
void uMyo_RF24_::setProtocolVersion(int version)
{
	protocol_version = version;
}
uint8_t uMyo_RF24_::getDeviceCount()
{
	return device_count;
}
int uMyo_RF24_::getBattery(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].batt_mv;
	return 0;
}
uint32_t uMyo_RF24_::getID(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].id;
	return 0;
}
uint8_t uMyo_RF24_::getDataID(uint8_t devidx)
{
	if(devidx < device_count) return devices[devidx].last_data_id;
	return 0;
}
float uMyo_RF24_::getMuscleLevel(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	if(protocol_version < 3) return devices[devidx].cur_spectrum[2] + 2*devices[devidx].cur_spectrum[3];
	return devices[devidx].device_avg_muscle_level;
}
float uMyo_RF24_::getMomentaryMuscleLevel(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	float lvl = devices[devidx].cur_spectrum[2] + 2*devices[devidx].cur_spectrum[3];
	return lvl;
}
void uMyo_RF24_::getSpectrum(uint8_t devidx, float *spectrum)
{
	if(devidx >= device_count) return;
	for(uint8_t x = 0; x < 4; x++) spectrum[x] = devices[devidx].cur_spectrum[x];
}
void uMyo_RF24_::getRawData(uint8_t devidx, int16_t *data)
{
	if(devidx >= device_count) return;
	for(uint8_t x = 0; x < 8; x++) data[x] = devices[devidx].raw_data[x];
}
float uMyo_RF24_::getPitch(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nzg = nz;
	sQ Qgs;
	Qgs.w = devices[devidx].Qsg.w;
	Qgs.x = -devices[devidx].Qsg.x;
	Qgs.y = -devices[devidx].Qsg.y;
	Qgs.z = -devices[devidx].Qsg.z;
	rotate_v(&Qgs, &nzg);
	return acos_f(v_dot(&nzg, &ny));
}
float uMyo_RF24_::getRoll(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nzg = nz;
	sQ Qgs;
	Qgs.w = devices[devidx].Qsg.w;
	Qgs.x = -devices[devidx].Qsg.x;
	Qgs.y = -devices[devidx].Qsg.y;
	Qgs.z = -devices[devidx].Qsg.z;
	rotate_v(&Qgs, &nzg);
	return atan2_f(nzg.z, nzg.x);
}
float uMyo_RF24_::getYaw(uint8_t devidx)
{
	if(devidx >= device_count) return 0;
	sV nyr = ny;
	rotate_v(&devices[devidx].Qsg, &nyr);
	return atan2_f(nyr.y, nyr.x);
}
uMyo_RF24_ uMyo;


