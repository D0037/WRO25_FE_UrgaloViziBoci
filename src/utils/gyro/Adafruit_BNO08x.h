/*!
 *  @file Adafruit_BNO08x.h
 *
 * 	I2C Driver for the Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_BNO08x_H
#define _ADAFRUIT_BNO08x_H

//#include "Arduino.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
//#include "BNO08x_SPI.hpp"
//#include <Adafruit_BusIO_Register.h>
//#include <Adafruit_I2CDevice.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>


/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT                                                       \
  9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x();
  ~Adafruit_BNO08x();

  bool begin_SPI(char* spidev, char* gpiochip, int rst_pin, int wake_pin, int int_pin, uint32_t speed, int sensor_id);

  void hardwareReset(void);
  bool wasReset(void);

  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  virtual bool _init(int32_t sensor_id);

  sh2_Hal_t
      _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};

#endif
