/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <zephyr.h>

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	u32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
	    h, min, s, ms);
	return buf;
}

void main(void)
{

	const char *const devname = DT_LABEL(DT_INST(0, microchip_mcp9600));
	struct device *dev = device_get_binding(devname);
	int rc;

	if (dev == NULL) {
		printf("Device %s not found.\n", devname);
		k_sleep(K_SECONDS(2));
		return;
	}

	struct sensor_value attr;
	if (sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
		SENSOR_ATTR_FULL_SCALE, &attr) < 0) {
		printf("Cannot set sensor scale.\n");
	}

	while (1) {
		struct sensor_value th, tc;

		rc = sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
		if (rc != 0) {
			printf("sensor_sample_fetch error: %d\n", rc);
			break;
		}

		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &th);
		if (rc != 0) {
			printf("sensor_channel_get error: %d\n", rc);
			break;
		}

		rc = sensor_sample_fetch_chan(dev, SENSOR_CHAN_DIE_TEMP);
		if (rc != 0) {
			printf("sensor_sample_fetch error: %d\n", rc);
			break;
		}

		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &tc);
		if (rc != 0) {
			printf("sensor_channel_get error: %d\n", rc);
			break;
		}

		printf("%s: %g %g C\n", now_str(),
		    sensor_value_to_double(&th), sensor_value_to_double(&tc));

		k_sleep(K_SECONDS(2));
	}
	printf("--> main is done\n");
}