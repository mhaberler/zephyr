/*
 * SPDX-License-Identifier: Apache-2.0
 * http://ww1.microchip.com/downloads/en/DeviceDoc/MCP960X-L0X-RL0X-Data-Sheet-20005426F.pdf
 * http://ww1.microchip.com/downloads/en/DeviceDoc/80000741B.pdf
 */
#include <stdio.h>
#define DT_DRV_COMPAT microchip_mcp9600

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

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
	return 0;
}

static int mcp9600_available(struct device *dev) {
	struct mcp9600_data *data = dev->driver_data;
  u8_t status;

#if 1
  if (mcp9600_reg_read(data, MCP9600_SENSOR_STATUS, &status, 1) < 0) {
    LOG_ERR("error reading status register");
  }
#endif
  if (status & MCP9600_SENSOR_STATUS_RDY_MASK) {
    return 1;
  } else {
    return 0;
  }
}

static int mcp9600_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct mcp9600_data *data = dev->driver_data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);
	printf("%s: fetch\n", dev->name);
#if 0
	return mcp9600_reg16_read(dev, MCP9600_DEVICE_ID, &data->reg_val);
#else
      return -EIO;
#endif
}

static int mcp9600_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	//const struct mcp9600_data *data = dev->driver_data;
	// int temp = mcp9600_temp_signed_from_reg(data->reg_val);

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	// val->val1 = temp / MCP9600_TEMP_SCALE_CEL;
	// temp -= val->val1 * MCP9600_TEMP_SCALE_CEL;
	// val->val2 = (temp * 1000000) / MCP9600_TEMP_SCALE_CEL;

	return 0;
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
				//
        //         return 0;
				//
        // case SENSOR_ATTR_SAMPLING_FREQUENCY:
				//
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


int mcp9600_init(struct device *dev)
{
	struct mcp9600_data *data = dev->driver_data;
	int rc = 0;

	data->i2c_master = device_get_binding(data->i2c_bus);
	if (!data->i2c_master) {
		LOG_DBG("%s: i2c master not found: %s", dev->name, data->i2c_bus);
		return -EINVAL;
	}
#if 1
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
#endif
 	rc = mcp9600_available(dev);


	printf("%s: revision=0x%x available=%d\n", dev->name, data->revision, rc  );
	LOG_DBG("%s: revision=0x%x\n", dev->name, data->revision);
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
