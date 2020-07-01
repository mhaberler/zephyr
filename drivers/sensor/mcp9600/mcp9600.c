/*
 * SPDX-License-Identifier: Apache-2.0
 * http://ww1.microchip.com/downloads/en/DeviceDoc/MCP960X-L0X-RL0X-Data-Sheet-20005426F.pdf
 * http://ww1.microchip.com/downloads/en/DeviceDoc/80000741B.pdf
 */
#include <stdio.h>
#define DT_DRV_COMPAT microchip_mcp9600

#include <errno.h>

#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>

#define MCP9600_FILTER_COEFFICIENT 0
#define MCP9600_THERMOCOUPLE_TYPE MCP9600_TYPE_K
#define MCP9600_RESOLUTION MCP9600_RES_18_BIT
#define MCP9600_BURST_SAMPLES MCP9600_SAMPLES_1
#define MCP9600_COLD_JUNC_RESOLUTION MCP9600RES_ZERO_POINT_0625
#define MCP9600_SHUTDOWN_MODE MCP9600_BURST

#include "mcp9600.h"

LOG_MODULE_REGISTER(MCP9600, CONFIG_SENSOR_LOG_LEVEL);

static int mcp9600_reg_read(struct mcp9600_data *data, u8_t start, void *buf,
    int size)
{
	return i2c_burst_read(data->i2c_master, data->i2c_slave_addr, start,
	    buf, size);
}

static int mcp9600_reg_write(struct mcp9600_data *data, u8_t reg, u8_t val)
{
	return i2c_reg_write_byte(data->i2c_master, data->i2c_slave_addr,
	    reg, val);
}

static int mcp9600_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct mcp9600_data *data = dev->driver_data;
	s16_t value;
	u8_t status;
	int rc;

	LOG_DBG("%s: fetch\n", dev->name);

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
        case SENSOR_CHAN_ALL:

		if ((rc = mcp9600_reg_read(dev->driver_data, MCP9600_SENSOR_STATUS,
			 &status, 1)) < 0) {
			LOG_DBG("%s: failed to read SENSOR_STATUS 0x%x", dev->name, rc);

			return rc;
		}
		if (!(status & (MCP9600_TH_UPDATE | MCP9600_BURST_COMPLETE_MASK))) {
			// no new sample available
			return 0;
		}
		if ((rc = mcp9600_reg_read(dev->driver_data, MCP9600_HOT_JUNC_TEMP,
			 &value, 2)) < 0) {
			LOG_DBG("%s: device unavailable", dev->name);
			return rc;
		}
		data->th = sys_be16_to_cpu(value);

		// start a burst sample
		mcp9600_reg_write(data, MCP9600_DEVICE_CONFIG,
		    (MCP9600_COLD_JUNC_RESOLUTION << 7) |
			(MCP9600_RESOLUTION << 5) |
			(MCP9600_BURST_SAMPLES << 2) |
			(MCP9600_SHUTDOWN_MODE & 3));
		if (chan == SENSOR_CHAN_AMBIENT_TEMP)
                  return 0;
                // fall through

	case SENSOR_CHAN_DIE_TEMP:
		if ((rc = mcp9600_reg_read(dev->driver_data, MCP9600_COLD_JUNC_TEMP,
			 &value, 2)) < 0) {
			LOG_DBG("%s: device unavailable", dev->name);
			return rc;
		}
		data->tc = sys_be16_to_cpu(value);
		break;
	default:
		return -EINVAL;
	}
}

static int mcp9600_channel_get(struct device *dev,
    enum sensor_channel chan,
    struct sensor_value *val)
{
	const struct mcp9600_data *data = dev->driver_data;
	s32_t calc_temp;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		calc_temp = ((data->th & 0x7fff) * 100) >> 4;
		if (data->th & 0x8000)
			calc_temp -= ((4096 * 100) >> 4);
		// calc_temp has a resolution of 0.01 degC.
		// So 5123 equals 51.23 degC.
		val->val1 = calc_temp / 100;
		val->val2 = (calc_temp % 100) * 10000;
		return 0;
		break;

	case SENSOR_CHAN_DIE_TEMP:
		calc_temp = ((data->tc & 0x0fff) * 100) >> 4;
		if (data->tc & 0x8000)
			calc_temp -= ((4096 * 100) >> 4);
		val->val1 = calc_temp / 100;
		val->val2 = (calc_temp % 100) * 10000;
		return 0;
		break;
	default:
		return -EINVAL;
	}
}

static int mcp9600_attr_set(struct device *dev, enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value *val)
{
	//const struct mcp9600_data *data = dev->driver_data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	printf("mcp9600_attr_set: attr=%d\n", attr);

	switch (attr) {
	// case SENSOR_ATTR_FULL_SCALE:
	//         return 0;
	// case SENSOR_ATTR_SAMPLING_FREQUENCY:
	//         return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api mcp9600_api_funcs = {
    .attr_set = mcp9600_attr_set,
    .sample_fetch = mcp9600_sample_fetch,
    .channel_get = mcp9600_channel_get,
};

struct init_regs {
	u8_t reg;
	u8_t val;
};

static const struct init_regs mcp9600_init_regs[] = {
    {MCP9600_SENSOR_STATUS, 0},
    {MCP9600_THERMO_SENSOR_CONFIG,
	(MCP9600_THERMOCOUPLE_TYPE << 4) |
	    (MCP9600_FILTER_COEFFICIENT & 0x7)},
    {MCP9600_DEVICE_CONFIG,
	(MCP9600_COLD_JUNC_RESOLUTION << 7) |
	    (MCP9600_RESOLUTION << 5) |
	    (MCP9600_BURST_SAMPLES << 2) |
	    (MCP9600_SHUTDOWN_MODE & 3)},
    {MCP9600_ALERT1_CONFIG, 0},
    {MCP9600_ALERT2_CONFIG, 0},
    {MCP9600_ALERT3_CONFIG, 0},
    {MCP9600_ALERT4_CONFIG, 0},
    {MCP9600_ALERT1_HYSTERESIS, 0},
    {MCP9600_ALERT2_HYSTERESIS, 0},
    {MCP9600_ALERT3_HYSTERESIS, 0},
    {MCP9600_ALERT4_HYSTERESIS, 0},
    {MCP9600_ALERT1_LIMIT, 0},
    {MCP9600_ALERT2_LIMIT, 0},
    {MCP9600_ALERT3_LIMIT, 0},
    {MCP9600_ALERT4_LIMIT, 0},
};

int mcp9600_init(struct device *dev)
{
	struct mcp9600_data *data = dev->driver_data;
	int rc = 0;

	data->i2c_master = device_get_binding(data->i2c_bus);
	if (!data->i2c_master) {
		LOG_DBG("%s: i2c master not found: %s",
		    dev->name, data->i2c_bus);
		return -EINVAL;
	}

	u16_t id;
	if (mcp9600_reg_read(data, MCP9600_DEVICE_ID, &id, sizeof(id)) < 0) {
		LOG_DBG("%s: Failed to read device ID register.", dev->name);
		return -EIO;
	}
	id = sys_be16_to_cpu(id);
	data->revision = (id & 0x00ff);
	u8_t device_id = (id >> 8);
	if (device_id != MCP9600_DEV_ID_UPPER) {
		LOG_DBG("%s: not an MCP9600 at 0x%x: device id=0x%x",
		    dev->name,
		    data->i2c_slave_addr, device_id);
		return -ENODEV;
	}
	rc = 0;
	for (int i = 0; i < (sizeof(mcp9600_init_regs) / sizeof(struct init_regs)); i++) {
		rc |= mcp9600_reg_write(data, mcp9600_init_regs[i].reg, mcp9600_init_regs[i].val);
	}

	if (rc) {
		LOG_DBG("%s: register setup failed\n", dev->name);
		return -EIO;
	}

	LOG_DBG("%s: revision=0x%x\n", dev->name, data->revision);
	mcp9600_reg_write(data, MCP9600_SENSOR_STATUS, 0);

	// start a burst
	mcp9600_reg_write(data, MCP9600_DEVICE_CONFIG,
	    (MCP9600_COLD_JUNC_RESOLUTION << 7) |
		(MCP9600_RESOLUTION << 5) |
		(MCP9600_BURST_SAMPLES << 2) |
		(MCP9600_SHUTDOWN_MODE & 3));

	return 0;
}

//static struct mcp9600_data mcp9600_data;
static struct mcp9600_data mcp9600_data = {
    .i2c_bus = DT_INST_BUS_LABEL(0),
    .i2c_slave_addr = DT_INST_REG_ADDR(0),
};

DEVICE_AND_API_INIT(mcp9600, DT_INST_LABEL(0), mcp9600_init,
    &mcp9600_data, NULL, POST_KERNEL,
    CONFIG_SENSOR_INIT_PRIORITY, &mcp9600_api_funcs);