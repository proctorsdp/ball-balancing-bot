#ifndef NUNCHUCK_H
#define NUNCHUCK_H

#include <Wire.h>

class nunchuck_state
{
  _Bool z;
  _Bool c;
  uint8_t x_joystick;
  uint8_t y_joystick;
  uint16_t x_tilt;
  uint16_t y_tilt;
  uint16_t z_tilt;
};

class Joystick {
  private: 
  uint8_t address = 0x52;
  
public:
  void Joystick(uint8_t address){
    this.address = address;
    
    uint8_t data[2];
    Wire.beginTransmission(address);
    Wire.write(0xF055);
    Wire.write(0xFB55);
    Wire.endTransmission();
  
    data[0] = 0xFB;
    data[1] = 0x55;
    i2c_write(i2c, 0x52, data, 2, 0);
  }

  struct nunchuck_state get_nunchuck_state() {
      struct nunchuck_state forReturn;
      uint8_t nunchuck_data[6];
               
      Wire.beginTransmission(address);
      Wire.write(0x00); // send command to remote to take new sample
      Wire.endTransmission();
      delay(12);
      
      i2c_write(i2c, 0x52, &data, 1, 0);
      msleep(12);                 // wait for new sample
    
      i2c_read(i2c, address, 0x00, nunchuck_data, 6);
      forReturn.x_joystick = nunchuck_data[0];
      forReturn.y_joystick = nunchuck_data[1];
      forReturn.x_tilt = (nunchuck_data[2] << 2 | (nunchuck_data[5] >> 6));
      forReturn.y_tilt = (nunchuck_data[3] << 2 | ((nunchuck_data[5] >> 4) & 0x03));
      forReturn.z_tilt = (nunchuck_data[4] << 2 | ((nunchuck_data[5] >> 2) & 0x03));
      forReturn.z = !(nunchuck_data[5] & 0x01);
      forReturn.c = !((nunchuck_data[5] >> 0x01) & 0x01);
      return forReturn;
  }
  void nunchuck_read(uint32_t* i2c, uint8_t address);
}



#endif
