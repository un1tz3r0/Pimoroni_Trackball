/*!
 * @file Trackball.cpp
 *
 *  @mainpage Pimoroni Trackball Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Pimoroni Trackball sensor
 *
 *  Designed specifically to work with the Pimoroni Trackball breakout board, 
 *  
 *  They are available here:
 *  ------> https://shop.pimoroni.com/products/trackball-breakout
 *
 *  A similar product is sold by Sparkfun, however it is more of a breakout board,
 *  so you have to watch the hall sensor signals yourself and count edges.
 *  
 *  These novel little input devices feature an (NXP?) embedded MCU that is 
 *  pre-programmed with I2C slave firmware to allow reading movement and setting,
 *  the duty-cycle of the LEDs. 2 pins are required to interface, plus ground and 
 *  3-5V power, and an optional interrupt pin (if not connected, interrupt status 
 *  is polled through the I2C bus.)
 *
 *  Cor3 LLC invests time and resources providing this open source code,
 *  please support Pimoroni and Cor3 LLC and open-source hardware by 
 *  purchasing products from companies like these and others who
 *
 *  @section author Author
 *
 *  V. Condino (Cor3 LLC) vic@cor3.llc un1tz3r0@gmail.com
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "Arduino.h"

#include <limits.h>
#include <math.h>

#include "Pimoroni_Trackball.h"

/*!
 *  @brief  Instantiates a new Pimoroni_Trackball class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  *theWire
 *          Wire object
 */
Pimoroni_Trackball::Pimoroni_Trackball(int32_t sensorID, uint8_t address,
                                 TwoWire *theWire) {
  _sensorID = sensorID;
  _address = address;
  _wire = theWire;
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool Pimoroni_Trackball::begin() {
//#if defined(ARDUINO_SAMD_ZERO) && (_address == BNO055_ADDRESS_A)
//#error                                                                         \
//    "On an arduino Zero, BNO055's ADR pin must be high. Fix that, then delete this line."
//  _address = BNO055_ADDRESS_B;
//#endif

  /* Enable I2C */
  _wire->begin();

  // BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  _wire->setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  /* Make sure we have the right device */
  uint16_t id = (uint16_t)read8(REG_CHIP_ID_L) | ((uint16_t)read8(REG_CHIP_ID_H) << 8);
  
  if (id != CHIP_ID) {
    delay(1000); // hold on for boot
    id = (uint16_t)read8(REG_CHIP_ID_L) | ((uint16_t)read8(REG_CHIP_ID_H) << 8);
    if (id != CHIP_ID) {
      return false; // still not? ok bail
    }
  }
  
  //delay(20);

  return true;
}

void Pimoroni_Trackball::setLEDs(uint32_t color)
{
	write8(REG_LED_RED, (color >> 24) && 0xff);
	write8(REG_LED_GRN, (color >> 16) && 0xff);
	write8(REG_LED_BLU, (color >> 8) && 0xff);
	write8(REG_LED_WHT, (color >> 0) && 0xff);
}

void Pimoroni_Trackball::getMotion(uint8_t &x, uint8_t &y)
{
	uint8_t left = read8(REG_LEFT);
	uint8_t right = read8(REG_RIGHT);
	uint8_t up = read8(REG_UP);
	uint8_t down = read8(REG_DOWN);
	x = right - left;
	y = up - down;
}

uint8_t Pimoroni_Trackball::getButton()
{
	return read8(REG_SWITCH) & MSK_SWITCH_STATE;
}

/*!
 *  @brief  Writes an 8 bit value over I2C
 */
bool Pimoroni_Trackball::write8(trackball_reg_t reg, byte value) {
  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)value);
#else
  _wire->send(reg);
  _wire->send(value);
#endif
  _wire->endTransmission();

  /* ToDo: Check for error! */
  return true;
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 */
byte Pimoroni_Trackball::read8(trackball_reg_t reg) {
  byte value = 0;

  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
#else
  _wire->send(reg);
#endif
  _wire->endTransmission();
  _wire->requestFrom(_address, (byte)1);
#if ARDUINO >= 100
  value = _wire->read();
#else
  value = _wire->receive();
#endif

  return value;
}

/*!
 *  @brief  Reads the specified number of bytes over I2C
 */
bool Pimoroni_Trackball::readLen(trackball_reg_t reg, byte *buffer,
                              uint8_t len) {
  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
#else
  _wire->send(reg);
#endif
  _wire->endTransmission();
  _wire->requestFrom(_address, (byte)len);

  for (uint8_t i = 0; i < len; i++) {
#if ARDUINO >= 100
    buffer[i] = _wire->read();
#else
    buffer[i] = _wire->receive();
#endif
  }

  /* ToDo: Check for errors! */
  return true;
}

