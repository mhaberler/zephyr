/*
 * lifted from https://github.com/sparkfun/SparkFun_MCP9600_Arduino_Library
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP9600_MCP9600_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP9600_MCP9600_H_

#include <errno.h>

#include <zephyr/types.h>
#include <device.h>
#include <drivers/sensor.h>
#include <sys/util.h>
#include <drivers/gpio.h>

#define MCP9600_DEV_ID_UPPER 0x40 //value of the upper half of the device ID register. lower half is used for device revision
#define MCP9600_DEV_RESOLUTION 0.0625 //device resolution (temperature in C that the LSB represents)
#define retryAttempts 3 //how many times to attempt to read a register from the thermocouple before giving up

// register pointers for various device functions
#define MCP9600_HOT_JUNC_TEMP  0x00
#define MCP9600_DELTA_JUNC_TEMP  0x01
#define MCP9600_COLD_JUNC_TEMP  0x02
#define MCP9600_RAW_ADC  0x03
#define MCP9600_SENSOR_STATUS  0x04
#define MCP9600_THERMO_SENSOR_CONFIG  0x05
#define MCP9600_DEVICE_CONFIG  0x06
#define MCP9600_ALERT1_CONFIG  0x08
#define MCP9600_ALERT2_CONFIG  0x09
#define MCP9600_ALERT3_CONFIG  0x0A
#define MCP9600_ALERT4_CONFIG  0x0B
#define MCP9600_ALERT1_HYSTERESIS  0x0C
#define MCP9600_ALERT2_HYSTERESIS  0x0D
#define MCP9600_ALERT3_HYSTERESIS  0x0E
#define MCP9600_ALERT4_HYSTERESIS  0x0F
#define MCP9600_ALERT1_LIMIT  0x10
#define MCP9600_ALERT2_LIMIT  0x11
#define MCP9600_ALERT3_LIMIT  0x12
#define MCP9600_ALERT4_LIMIT  0x13
#define MCP9600_DEVICE_ID  0x20

// MCP9600_SENSOR_STATUS
#define MCP9600_SENSOR_STATUS_RDY_MASK BIT(6)

// Thermocouple type
#define MCP9600_TYPE_K  0b000
#define MCP9600_TYPE_J  0b001
#define MCP9600_TYPE_T  0b010
#define MCP9600_TYPE_N  0b011
#define MCP9600_TYPE_S  0b100
#define MCP9600_TYPE_E  0b101
#define MCP9600_TYPE_B  0b110
#define MCP9600_TYPE_R  0b111

// Ambient resolution
#define MCP9600RES_ZERO_POINT_0625  0
#define MCP9600RES_ZERO_POINT_25  1

// Thermocouple resolution
#define MCP9600_RES_18_BIT  0b00
#define MCP9600_RES_16_BIT  0b01
#define MCP9600_RES_14_BIT  0b10
#define MCP9600_RES_12_BIT  0b11

// Burst samples
#define MCP9600_SAMPLES_1  0b000
#define MCP9600_SAMPLES_2  0b001
#define MCP9600_SAMPLES_4  0b010
#define MCP9600_SAMPLES_8  0b011
#define MCP9600_SAMPLES_16  0b100
#define MCP9600_SAMPLES_32  0b101
#define MCP9600_SAMPLES_64  0b110
#define MCP9600_SAMPLES_128  0b111

// Shutdown mode
#define MCP9600_NORMAL  0x00
#define MCP9600_SHUTDOWN  0x01
#define MCP9600_BURST  0x02

// struct mcp9600_data {
// 	struct device *i2c_master;
// 	u16_t reg_val;
//   u8_t revision;
// };

struct mcp9600_data {
	struct device *i2c_master;
	const char *i2c_bus;
	u16_t i2c_slave_addr;
	u16_t reg_val;
	u8_t revision;
};

//int mcp9600_reg_read(struct device *dev, u8_t reg, u16_t *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP9600_MCP9600_H_ */
