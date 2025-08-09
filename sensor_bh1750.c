/*
 * MIT License
 *
 * Copyright (c) 2025 huxiangjs
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
#include "sensor_bh1750.h"

static const char *TAG = "SENSOR-BH1750";

#define INTERVAL_TIME		200	/* 200ms */

static bool bh1750_active;
static uint16_t bh1750_data;

static inline bool sensor_bh1750_byte_write(uint8_t data)
{
	return i2c_bus_write(DEFAULT_BH1750_ADDR, &data, 1);
}

static inline bool sensor_bh1750_2bytes_read(uint16_t *data)
{
	return i2c_bus_read(DEFAULT_BH1750_ADDR, data, 2);
}

/* Get the ambient light brightness, unit: lx */
static bool sensor_bh1750_get_ambient_light(void)
{
	bool ret;
	uint16_t data;

	/*
	 * Start measurement at 1lx resolution.
	 * Measurement Time is typically 120ms.
	 * It is automatically set to Power Down mode after measurement.
	 */
	ret = sensor_bh1750_byte_write(0x20);
	if (ret != true) {
		ESP_LOGE(TAG, "Failed to start measurement");
		return false;
	}

	vTaskDelay(pdMS_TO_TICKS(200));

	ret = sensor_bh1750_2bytes_read(&data);
	if (ret) {
		/*
		 * Calculate the lx value according to the manual
		 * if the accuracy is 1, divide it by 1.2
		 */
		bh1750_data = (uint16_t)(data / 1.2);
	} else {
		ESP_LOGE(TAG, "Failed to read data");
		return false;
	}

	return true;
}

static void sensor_bh1750_task(void *pvParameters)
{
	struct event_bus_msg msg = { EVENT_BUS_SENSOR_BRIGHTNESS_UPDATED, 0, 0 };
	bool ret;

	vTaskDelay(pdMS_TO_TICKS(40));

	/* Power on */
	ret = sensor_bh1750_byte_write(0x01);
	if (ret != true) {
		ESP_LOGE(TAG, "Failed to power on");
		goto err;
	}

	while(1) {
		if (sensor_bh1750_get_ambient_light()) {
			bh1750_active = true;
			ESP_LOGD(TAG, "Light Intensity: %5u lx", bh1750_data);
			msg.param2 = bh1750_data;
			event_bus_send(&msg);
		}
		vTaskDelay(pdMS_TO_TICKS(INTERVAL_TIME));
	}

err:
	vTaskDelete(NULL);
}

/* Unit: lx */
uint16_t sensor_bh1750_get_brightness(void)
{
	return bh1750_data;
}

bool sensor_bh1750_is_active(void)
{
	return bh1750_active;
}

void sensor_bh1750_init(void *p)
{
	int ret;

	ret = xTaskCreate(sensor_bh1750_task, "BH1750 Task", 1024,
			  NULL, tskIDLE_PRIORITY, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);
}
