#ifndef NUNCHUCK_H
#define NUNCHUCK_H

#include "tiva_c.h"

uint8_t address = 0x52;

struct nunchuck_state
{
	_Bool z;
	_Bool c;
	uint8_t x_joystick;
	uint8_t y_joystick;
	uint16_t x_tilt;
	uint16_t y_tilt;
	uint16_t z_tilt;
};

void nunchuck_init(uint32_t* i2c, uint32_t sys_clock);
struct nunchuck_state get_nunchuck_state(uint32_t* i2c, uint8_t address) {
		struct nunchuck_state forReturn;
		uint8_t nunchuck_data[6];
		uint8_t data;

		data = 0x00;								// send command to remote to take new sample
		i2c_write(i2c, 0x52, &data, 1, 0);
		msleep(12);									// wait for new sample

		i2c_read(i2c, address, 0x00, nunchuck_data, 6);
		forReturn.x_joystick = nunchuck_data[0];
		forReturn.y_joystick = nunchuck_data[1];
		forReturn.x_tilt = (nunchuck_data[2] << 2 | (nunchuck_data[5] >> 6));
		forReturn.y_tilt = (nunchuck_data[3] << 2 | ((nunchuck_data[5] >> 4) & 0x03));
		forReturn.z_tilt = (nunchuck_data[4] << 2 | ((nunchuck_data[5] >> 2) & 0x03));
		forReturn.z = !(nunchuck_data[5] & 0x01);
		forReturn.c = !((nunchuck_data[5] >> 0x01) & 0x01);
		return forReturn;
};
void nunchuck_read(uint32_t* i2c, uint8_t address);


#endif
