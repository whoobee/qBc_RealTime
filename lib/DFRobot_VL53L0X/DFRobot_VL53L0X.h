/*!
 * @file DFRobot_VL53L0X.h
 * @brief DFRobot's Laser rangefinder library. Local fork patched to use Wire2
 *        (Teensy 4.1 SDA2/SCL2 on pins 25/24). Upstream: github.com/DFRobot/DFRobot_VL53L0X
 */

#ifndef __DFRobot_VL53L0X_H
#define __DFRobot_VL53L0X_H

#include <Arduino.h>
#include <Wire.h>

#define VL53L0X_DEF_I2C_ADDR 0x29

// Per-instance state. Was a file-scope global in upstream, which broke
// multi-sensor use — every object hit the same I2C address. Lifted into
// the class so each VL53L0X has its own state.
struct sVL53L0X_DetailedData_t {
  unsigned char i2cDevAddr;
  uint8_t mode;
  uint8_t precision;
  unsigned char originalData[16];
  uint16_t ambientCount;
  uint16_t signalCount;
  uint16_t distance;
  uint8_t status;
};

class DFRobot_VL53L0X{
public:
  typedef enum {eDISABLE = 0, eENABLE = !eDISABLE} eFunctionalState;
  typedef enum {eHigh = 0, eLow = !eHigh} ePrecisionState;
  typedef enum {eSingle = 0, eContinuous = !eSingle} eModeState;
  DFRobot_VL53L0X();
  ~DFRobot_VL53L0X();
  void begin(uint8_t addr = VL53L0X_DEF_I2C_ADDR);
  void setMode(eModeState mode, ePrecisionState precision);
  void start();
  void stop();
  float getDistance();
  uint16_t getAmbientCount();
  uint16_t getSignalCount();
  uint8_t getStatus();
private:
	uint16_t _distance;
	sVL53L0X_DetailedData_t _detailedData;
	void writeByteData(unsigned char Reg, unsigned char byte);
	uint8_t readByteData(unsigned char Reg);
	void writeData(unsigned char Reg ,unsigned char *buf, unsigned char Num);
	void readData(unsigned char Reg, unsigned char Num);
	void setDeviceAddress(uint8_t newAddr);
	void highPrecisionEnable(eFunctionalState NewState);
	void dataInit();
	void readVL53L0X();
};

#endif
