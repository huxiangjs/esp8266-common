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
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct i2c_dev_init {
	uint8_t addr;
	char *name;
	/*
	 * Optional: preparation, run by i2c_bus_init() before the bus
	 * scan: put the device in a state in which the scan can find
	 * it (e.g. the SGP30, which the scan only sees in its sleep
	 * state, is soft-reset there). NULL for devices that need
	 * none.
	 */
	void (*init)(void *p);
	/* Start the device's polling task (run for the devices the scan found) */
	void (*start)(void *p);
	void *p;
	/*
	 * Optional: request the device's polling task to stop. It is
	 * non-blocking: the task notices the request at a safe point
	 * (between two I2C transactions) and exits on its own;
	 * i2c_bus_stop() waits for that. NULL for devices without a
	 * polling task.
	 */
	void (*stop)(void *p);
	/* Optional: whether the device's polling task is (still) running. */
	bool (*is_active)(void *p);
};

void i2c_bus_init(uint8_t sda_gpio_num, uint8_t scl_gpio_num, const struct i2c_dev_init *list, uint16_t n);

/*
 * Stop the polling tasks of all the devices of the list i2c_bus_init()
 * was given: request every task to stop first, then wait for them all
 * to exit, so no I2C transaction is in flight when this returns.
 *
 * Used before a warm reboot (esp_restart()), which does not fully reset
 * the I2C bus: rebooting with a transaction in progress can leave the
 * bus locked.
 */
void i2c_bus_stop(void);
bool i2c_bus_read(uint8_t addr, void *data, uint16_t size);
bool i2c_bus_write(uint8_t addr, void *data, uint16_t size);
bool i2c_bus_in_read(uint8_t addr, uint8_t in_addr, void *data, uint16_t size);
bool i2c_bus_in_write(uint8_t addr, uint8_t in_addr, void *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_BUS_H_ */
