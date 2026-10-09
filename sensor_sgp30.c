/*
 * MIT License
 *
 * Copyright (c) 2026 huxiangjs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>

#include "event_bus.h"
#include "i2c_bus.h"
#include "sensor_sgp30.h"

static const char *TAG = "SENSOR-SGP30";

#define INTERVAL_TIME		1000	/* 1000ms, 1Hz sampling rate */

/* SGP30 16-bit commands */
#define SGP30_CMD_IAQ_INIT		0x2003	/* IAQ baseline init */
#define SGP30_CMD_MEASURE_GAS		0x2008	/* Measure tVOC and CO2eq */
#define SGP30_CMD_GET_FEATURESET	0x202F	/* Get feature set version */
#define SGP30_CMD_GET_TVOC_BASELINE	0x20B3	/* Get tVOC inceptive baseline */
#define SGP30_CMD_SET_TVOC_BASELINE	0x2077	/* Set tVOC inceptive baseline */

/* The inceptive baseline feature is available from feature set 0x21 */
#define SGP30_FEATURESET_INCEPTIVE	0x21

static bool sgp30_active;
static uint16_t sgp30_tvoc;		/* tVOC, unit: ppb */
static uint16_t sgp30_co2;		/* CO2eq, unit: ppm */

/*
 * CRC8, polynomial 0x31 (x^8+x^5+x^4+1), init 0xFF,
 * no reflection, no final XOR. The CRC covers the two
 * data bytes which it follows.
 */
static uint8_t sensor_sgp30_crc8(const uint8_t *buffer, uint8_t size)
{
	uint8_t i, byte;
	uint8_t crc = 0xFF;

	for (byte = 0; byte < size; byte++) {
		crc ^= buffer[byte];
		for(i = 8; i > 0; --i)
			crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
	}

	return crc;
}

/* Send a 16-bit command */
static bool sensor_sgp30_cmd(uint16_t cmd)
{
	uint8_t data[] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xff) };

	return i2c_bus_write(DEFAULT_SGP30_ADDR, data, sizeof(data));
}

/*
 * Send a 16-bit command with a 16-bit argument (two data bytes
 * followed by one CRC byte).
 */
static bool sensor_sgp30_cmd_with_arg(uint16_t cmd, uint16_t arg)
{
	uint8_t data[5];

	data[0] = (uint8_t)(cmd >> 8);
	data[1] = (uint8_t)(cmd & 0xff);
	data[2] = (uint8_t)(arg >> 8);
	data[3] = (uint8_t)(arg & 0xff);
	data[4] = sensor_sgp30_crc8(&data[2], 2);

	return i2c_bus_write(DEFAULT_SGP30_ADDR, data, sizeof(data));
}

/*
 * Read n data words from the sensor. A word consists of two data
 * bytes (MSB first) followed by one CRC byte. The CRC of every
 * word is checked.
 */
static bool sensor_sgp30_read_words(uint16_t *words, uint8_t n)
{
	uint8_t buffer[16];
	uint8_t i, j;

	if (n > 5)
		return false;

	if (!i2c_bus_read(DEFAULT_SGP30_ADDR, buffer, (uint16_t)(n * 3)))
		return false;

	for (i = 0, j = 0; i < n; i++, j += 3) {
		if (sensor_sgp30_crc8(&buffer[j], 2) != buffer[j + 2]) {
			ESP_LOGE(TAG, "CRC8 mismatch");
			return false;
		}
		words[i] = (uint16_t)((buffer[j] << 8) | buffer[j + 1]);
	}

	return true;
}

/*
 * Get the feature set version and the product type.
 * The product type is in bits [15:12], 0 for the SGP30.
 * The feature set version is in bits [7:0].
 */
static bool sensor_sgp30_get_featureset(uint16_t *version, uint8_t *product_type)
{
	uint16_t word;
	bool ret;

	ret = sensor_sgp30_cmd(SGP30_CMD_GET_FEATURESET);
	if (!ret)
		return false;
	vTaskDelay(pdMS_TO_TICKS(10));
	ret = sensor_sgp30_read_words(&word, 1);
	if (!ret)
		return false;

	*product_type = (uint8_t)((word >> 12) & 0x0f);
	*version = word & 0x00ff;

	return true;
}

/*
 * Reset the IAQ baselines. The command has to be sent after every
 * power-up or soft reset, and the measurement commands have to be
 * sent in regular intervals of 1s after that.
 */
static bool sensor_sgp30_iaq_init(void)
{
	bool ret;

	ret = sensor_sgp30_cmd(SGP30_CMD_IAQ_INIT);
	if (!ret)
		return false;
	vTaskDelay(pdMS_TO_TICKS(10));

	return true;
}

/*
 * Activate the tVOC inceptive baseline (feature set 0x21 and later).
 * It improves the tVOC accuracy on the very first start-up under
 * bad air condition.
 */
static bool sensor_sgp30_set_inceptive_baseline(uint16_t fs_version)
{
	uint16_t tvoc_baseline;
	bool ret;

	if (fs_version < SGP30_FEATURESET_INCEPTIVE)
		return true;

	ret = sensor_sgp30_cmd(SGP30_CMD_GET_TVOC_BASELINE);
	if (!ret)
		return false;
	vTaskDelay(pdMS_TO_TICKS(10));
	ret = sensor_sgp30_read_words(&tvoc_baseline, 1);
	if (!ret)
		return false;

	if (tvoc_baseline == 0)
		tvoc_baseline = 0x8000;

	return sensor_sgp30_cmd_with_arg(SGP30_CMD_SET_TVOC_BASELINE,
			tvoc_baseline);
}

/*
 * Measure the tVOC (ppb) and CO2eq (ppm) concentrations.
 * The response words are in the order CO2eq and tVOC.
 * During the first 15s after iaq_init the sensor returns fixed
 * values: 400 ppm CO2eq and 0 ppb tVOC.
 */
static bool sensor_sgp30_measure_iaq(void)
{
	uint16_t words[2];

	if (!sensor_sgp30_cmd(SGP30_CMD_MEASURE_GAS))
		return false;
	vTaskDelay(pdMS_TO_TICKS(20));
	if (!sensor_sgp30_read_words(words, 2))
		return false;

	sgp30_co2 = words[0];
	sgp30_tvoc = words[1];

	return true;
}

static void sensor_sgp30_task(void *pvParameters)
{
	struct event_bus_msg msg;
	uint16_t fs_version;
	uint8_t product_type = 0xff;

	vTaskDelay(pdMS_TO_TICKS(40));

	if (!sensor_sgp30_get_featureset(&fs_version, &product_type)) {
		ESP_LOGE(TAG, "Failed to get feature set");
		goto err;
	}
	if (product_type != 0) {
		ESP_LOGE(TAG, "Invalid product type: 0x%02x", product_type);
		goto err;
	}
	ESP_LOGI(TAG, "Feature set: 0x%02x", fs_version);

	if (!sensor_sgp30_iaq_init()) {
		ESP_LOGE(TAG, "Failed to init IAQ baselines");
		goto err;
	}
	if (!sensor_sgp30_set_inceptive_baseline(fs_version)) {
		ESP_LOGE(TAG, "Failed to set tVOC inceptive baseline");
		goto err;
	}

	while(1) {
		if (sensor_sgp30_measure_iaq()) {
			ESP_LOGI(TAG, "tVOC: %5u ppb, CO2eq: %5u ppm",
				 sgp30_tvoc, sgp30_co2);
			msg.type = EVENT_BUS_SENSOR_VOC_UPDATED;
			msg.param1 = 0;
			msg.param2 = sgp30_tvoc;
			event_bus_send(&msg);
			msg.type = EVENT_BUS_SENSOR_CO2_UPDATED;
			msg.param1 = 0;
			msg.param2 = sgp30_co2;
			event_bus_send(&msg);
		}
		vTaskDelay(pdMS_TO_TICKS(INTERVAL_TIME - 20));
	}

err:
	vTaskDelete(NULL);
}

/* Unit: ppb */
uint16_t sensor_sgp30_get_tvoc(void)
{
	return sgp30_tvoc;
}

/* Unit: ppm */
uint16_t sensor_sgp30_get_co2(void)
{
	return sgp30_co2;
}

bool sensor_sgp30_is_active(void)
{
	return sgp30_active;
}

void sensor_sgp30_init(void *p)
{
	int ret;

	ret = xTaskCreate(sensor_sgp30_task, "SGP30 Task", 1024,
			  NULL, tskIDLE_PRIORITY, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);

	sgp30_active = true;
}
