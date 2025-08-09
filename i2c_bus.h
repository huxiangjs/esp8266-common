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

#ifndef __I2C_BUS_H_
#define __I2C_BUS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct i2c_dev_init {
	uint8_t addr;
	char *name;
	void (*init)(void *p);
	void *p;
};

void i2c_bus_init(uint8_t sda_gpio_num, uint8_t scl_gpio_num, const struct i2c_dev_init *list, uint16_t n);
bool i2c_bus_read(uint8_t addr, void *data, uint16_t size);
bool i2c_bus_write(uint8_t addr, void *data, uint16_t size);
bool i2c_bus_in_read(uint8_t addr, uint8_t in_addr, void *data, uint16_t size);
bool i2c_bus_in_write(uint8_t addr, uint8_t in_addr, void *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_BUS_H_ */
