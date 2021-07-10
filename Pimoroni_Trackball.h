/*!
 *  @file Pimoroni_Trackball.h
 *
 *  This is a library for the Pimoroni Tackball breakout.
 *
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *  This driver does not implement support for the interrupt signal from the
 *  trackball breakout, that's left as an excersise for the reader.
 *
 *  - Victor Condino (Cor3 LLC)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __PIMORONI_TRACKBALL_H__
#define __PIMORONI_TRACKBALL_H__

#include "Arduino.h"
#include <Wire.h>

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          Pimoroni Trackball breakout's embedded Nuvoton mcu via I2C
 */
class Pimoroni_Trackball
{
public:
  enum {
		I2C_ADDRESS = 0x0A,
		I2C_ADDRESS_ALTERNATIVE = 0x0B
	};

  enum {
		CHIP_ID = 0xBA11,
		VERSION = 1
	};

  enum {
		REG_LED_RED = 0x00,
		REG_LED_GRN = 0x01,
		REG_LED_BLU = 0x02,
		REG_LED_WHT = 0x03,

		REG_LEFT = 0x04,
		REG_RIGHT = 0x05,
		REG_UP = 0x06,
		REG_DOWN = 0x07,
		REG_SWITCH = 0x08,
		MSK_SWITCH_STATE = 0b10000000,

		REG_USER_FLASH = 0xD0,
		REG_FLASH_PAGE = 0xF0,
		REG_INT = 0xF9,
		MSK_INT_TRIGGERED = 0b00000001,
		MSK_INT_OUT_EN = 0b00000010,
		REG_CHIP_ID_L = 0xFA,
		RED_CHIP_ID_H = 0xFB,
		REG_VERSION = 0xFC,
		REG_I2C_ADDR = 0xFD,
		REG_CTRL = 0xFE,
		
		MSK_CTRL_SLEEP = 0b00000001,
		MSK_CTRL_RESET = 0b00000010,
		MSK_CTRL_FREAD = 0b00000100,
		MSK_CTRL_FWRITE = 0b00001000
	};
	
	typedef uint8_t trackball_reg_t;

  Pimoroni_Trackball(uint8_t address = I2C_ADDRESS,
                  TwoWire *theWire = &Wire);

  bool begin();
  void setLEDs(uint32_t color);
  void getMotion(uint8_t &x, uint8_t &y);
  uint8_t getButton();
  
private:
  byte read8(trackball_reg_t);
  bool readLen(trackball_reg_t, byte *buffer, uint8_t len);
  bool write8(trackball_reg_t, byte value);

  uint8_t _address;
  TwoWire *_wire;
};

#endif
