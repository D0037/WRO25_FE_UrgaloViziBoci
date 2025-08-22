/*!
 *  @file Adafruit_BNO08x.cpp
 *
 *  @mainpage Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the BNO08x 9-DOF Orientation IMU Fusion
 * Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/product/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

//#include "Arduino.h"
//#include <Wire.h>

#include "Adafruit_BNO08x.h"
#include "BNO08x_SPI.hpp"
#include <chrono>
#include <boost/stacktrace.hpp>

static BNO08x_SPI *spi_dev = NULL; ///< Pointer to SPI bus interface
static int8_t _int_pin, _reset_pin;

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;
static auto start_time = std::chrono::high_resolution_clock::now();

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void spihal_close(sh2_Hal_t *self);
static int spihal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hal_hardwareReset(void);

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 * @param reset_pin The arduino pin # connected to the BNO Reset pin
 */
Adafruit_BNO08x::Adafruit_BNO08x() {}

/**
 * @brief Destroy the Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::~Adafruit_BNO08x(void) {
  // if (temp_sensor)
  //   delete temp_sensor;
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  int_pin The arduino pin # connected to BNO08x INT
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return true if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_SPI(
          char* spidev, char* gpiochip, int rst_pin, int wake_pin, int int_pin, uint32_t speed, int sensor_id)
  {

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  
  spi_dev = new BNO08x_SPI(spidev, gpiochip, rst_pin, wake_pin, int_pin, speed);
  if (!spi_dev->begin()) {
    return false;
  }

  _HAL.open = spihal_open;
  _HAL.close = spihal_close;
  _HAL.read = spihal_read;
  _HAL.write = spihal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::_init(int32_t sensor_id) {
  int status;

  hardwareReset();

  uint8_t data[] = {5, 0, 1, 0, 1};
  //spi_dev->write(data, 5);
  /*for (int i = 0; i < 2; i++) {
    /*uint8_t header[4];
    spi_dev->read(header, 4, 0x00);

    header[1] ^= header[1] & 0x80;
    uint16_t size = header[0] | header[1] << 8;
    std::cout << "packet size: " << size << "\n";

    uint8_t buff[0x300];
    memset(buff, 0x00, sizeof(buff));
    spi_dev->read(buff, 0x300, 0x00);

    for (char byte : buff) {
      std::cout << byte;
    }

    for (char byte : buff) {
      std::cout << (int) byte << " ";
    }
    uint8_t buff2[] = {0x06, 0x00, 0x02, 0x01, 0xF9, 0x00};
    //spi_dev->write(buff2, 6);    
  }*/

  


  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    std::cout << "SH2 not OK\n";
    return false;
  }
  std::cout << "SH2 OK\n";

  //hardwareReset();


  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    std::cout << "Failed to get prduct id! Status: " << status << "\n";
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void Adafruit_BNO08x::hardwareReset(void) {
  hal_hardwareReset();
}

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool Adafruit_BNO08x::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool Adafruit_BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;
                          

  config.reportInterval_us = interval_us;
  std::cout << "Enabling sensor: " << (int)sensorId
            << " with interval: " << interval_us << "\n";
  int status = sh2_setSensorConfig(sensorId, &config);
  std::cout << "Sensor config status: " << status << "\n";

  if (status != SH2_OK) {
    std::cout << "Failed to set sensor config for sensor: " << (int)sensorId
              << ", status: " << status << "\n";
    return false;
  }

  return true;
}

static int spihal_open(sh2_Hal_t *self) {
  // Serial.println("SPI HAL open");

  spi_dev->wait_for_int();

  return 0;
}

static void spihal_close(sh2_Hal_t *self) {
  // Serial.println("SPI HAL close");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("SPI HAL read");

  uint16_t packet_size = 0;

  /*if (!spi_dev->read(pBuffer, 4, 0x00)) {
    return 0;
  }

  */
  uint8_t buff[0x500];
  memset(buff, 0x00, sizeof(buff));

  if (!spi_dev->read(buff, sizeof(buff), 0x00) < 0) {
    return 0;
  }

  // Determine amount to read
  packet_size = (uint16_t)buff[0] | (uint16_t)buff[1] << 8;

  if (packet_size > len) {
    return 0;
  }

  for (unsigned int i = 0; i < packet_size & ~0x8000; i++) {
    pBuffer[i] = buff[i];
  }

  /*std::cout << "Received packet: ";
  for (unsigned int i = 0; i < packet_size; i++) {
    std::cout << (int) pBuffer[i] << " ";
  }*/
  
  *t_us = hal_getTimeUs(NULL);
  //std::cout << "Timestamp: " << std::dec << *t_us << std::hex << " us\n";
  return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  // Serial.print("SPI HAL write packet size: ");
  // Serial.println(len);

  //std::cout << boost::stacktrace::stacktrace() << "\n";

  spi_dev->write(pBuffer, len);
/*  std::cout << "Wrote packet: ";
  for (unsigned int i = 0; i < len; i++) {
    std::cout << (int) pBuffer[i] << " ";
  }
  std::cout << "\n";*/


  return len;
}

/**************************************** HAL interface
 * ***********************************************************/

static void hal_hardwareReset(void) {
  if (spi_dev->hard_reset() < 0) {
    std::cerr << "Could not reset\n";
    exit(-1);
  }
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  std::chrono::duration<double, std::micro> dt;
  dt = std::chrono::high_resolution_clock::now() - start_time;

  return dt.count();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    std::cout << "Reset!\n";
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    std::cout << "BNO08x - Error decoding sensor event\n";
    _sensor_value->timestamp = 0;
    return;
  }
}
