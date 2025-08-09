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
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/i2c.h>

#include "i2c_bus.h"

bool i2c_bus_read(uint8_t addr, void *data, uint16_t size)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, true);
	i2c_master_read(cmd, (uint8_t *)data, size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	return (bool)(ret == ESP_OK);
}

bool i2c_bus_write(uint8_t addr, void *data, uint16_t size)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write(cmd, (uint8_t *)data, size, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	return (bool)(ret == ESP_OK);
}

bool i2c_bus_in_read(uint8_t addr, uint8_t in_addr, void *data, uint16_t size)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, in_addr, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, true);
	i2c_master_read(cmd, (uint8_t *)data, size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	return (bool)(ret == ESP_OK);
}

bool i2c_bus_in_write(uint8_t addr, uint8_t in_addr, void *data, uint16_t size)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, in_addr, true);
	i2c_master_write(cmd, (uint8_t *)data, size, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
	i2c_cmd_link_delete(cmd);

	return (bool)(ret == ESP_OK);
}

static inline bool i2c_bud_dev_is_active(uint8_t addr)
{
	uint8_t data;

	return i2c_bus_read(addr, &data, 1);
}

static void i2c_bud_scan(bool *active)
{
	int i, j;
	uint8_t addr;

	printf("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
	for (i = 0; i < 8; i++) {
		printf("%02X ", i<<4);
		for (j = 0; j <= 0xF; j++) {
			addr = i << 4 | j;
			if (i2c_bud_dev_is_active(addr)) {
				if (active)
					active[addr] = true;
				printf("%02X ", addr);
			} else {
				printf("-- ");
			}
		}
		printf("\n");
	}
}

void i2c_bus_init(uint8_t sda_gpio_num, uint8_t scl_gpio_num, const struct i2c_dev_init *list, uint16_t n)
{
	uint8_t index;
	bool active[0x7f];
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda_gpio_num,
		.sda_pullup_en = 1,
		.scl_io_num = scl_gpio_num,
		.scl_pullup_en = 1,
		.clk_stretch_tick = 300,	/* 300 ticks, Clock stretch is about 210us */
	};
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode));
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

	/* Find the address on the bus where there is an ACK */
	memset(active, 0, sizeof(active));
	i2c_bud_scan(active);

	/* Init device */
	for (index = 0; index < n; index++) {
		if (!active[list[index].addr])
			continue;
		printf("Init: %s\n", list[index].name);
		list[index].init(list[index].p);
	}
}
