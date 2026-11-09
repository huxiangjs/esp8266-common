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
#include <string.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/uart.h>

#include "event_bus.h"

static const char *TAG = "HLW8032";

static uart_port_t hlw8032_uart;
static TaskHandle_t handle;
static bool task_running;
static bool probed;

/*
 * physical coefficients, from the front-end circuit:
 * V_COEF = divider ratio/1000 = (470K*4)/(1K*1000) = 1.88
 * I_COEF = 1/(R_sample*1000) = 1/(0.001*1000) = 1.0, R_sample is the current sampling resistor
 */
#define HLW8032_V_COEF 1.88f
#define HLW8032_I_COEF 1.0f

/* PFcnt = k*65536 + n: n = PF REG value, k = toggle count of update REG bit7 (PF carry) */
static uint32_t hlw8032_pf_k;
static uint8_t  hlw8032_pf_carry = 0xFF;

static float last_v;
static float last_i;
static float last_p;
static float last_energy;

static inline uint32_t hlw8032_be24(const uint8_t *p)
{
	return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

/*
 * HLW8032 frame (24 bytes, multi-byte REGs are big-endian):
 * [0] state [1] check [2:5] V-param [5:8] V [8:11] I-param [11:14] I
 * [14:17] P-param [17:20] P [20] update [21:23] PF [23] checksum
 */
static inline bool hlw8032_frame_update(const uint8_t *d)
{
	uint32_t v_param, v_reg, i_param, i_reg, p_param, p_reg, sum, pf_cnt;
	float v, i, p, s, energy;
	size_t k;
	uint8_t state;
	uint8_t carry;

	/* state REG can only be 0x55/0xAA/0xFx, first filter against false sync */
	state = d[0];
	if (state != 0x55 && state != 0xAA && (state & 0xF0) != 0xF0) {
		ESP_LOGE(TAG, "State invalid");
		return false;
	}

	/* checksum = low 8 bits of the sum of all REGs except state/check/checksum */
	sum = 0;
	for (k = 2; k < 23; k++)
		sum += d[k];
	if ((sum & 0xFF) != d[23]) {
		ESP_LOGE(TAG, "Checksum failed");
		return false;
	}

	v_param = hlw8032_be24(d + 2);
	v_reg   = hlw8032_be24(d + 5);
	i_param = hlw8032_be24(d + 8);
	i_reg   = hlw8032_be24(d + 11);
	p_param = hlw8032_be24(d + 14);
	p_reg   = hlw8032_be24(d + 17);

	/* rms values: V = V-param/V*V_COEF, I = I-param/I*I_COEF, P = P-param/P*V_COEF*I_COEF */
	v = v_reg ? (float)v_param / v_reg * HLW8032_V_COEF : 0;
	i = i_reg ? (float)i_param / i_reg * HLW8032_I_COEF : 0;
	p = p_reg ? (float)p_param / p_reg * HLW8032_V_COEF * HLW8032_I_COEF : 0;
	s = v * i;

	/* count update REG bit7 (PF carry) toggles to accumulate the full PFcnt */
	carry = d[20] >> 7;
	if (hlw8032_pf_carry != 0xFF && carry != hlw8032_pf_carry)
		hlw8032_pf_k++;
	hlw8032_pf_carry = carry;
	pf_cnt = hlw8032_pf_k * 65536 + (((uint32_t)d[21] << 8) | d[22]);

	/* energy(kWh) = PFcnt/pulses_per_kWh, pulses_per_kWh = 1e9*3600/(P-param*V_COEF*I_COEF) */
	energy = p_param ? (float)pf_cnt * p_param * HLW8032_V_COEF * HLW8032_I_COEF / (1e9f * 3600.0f) : 0;

	/* state check */
	if (state == 0xAA) {
		last_v = 0.0f;
		last_i = 0.0f;
		last_p = 0.0f;
	} else if (state == 0x55) {
		last_v = ((d[20] >> 6) & 0x1) ? v : last_v;
		last_i = ((d[20] >> 5) & 0x1) ? i : last_i;
		last_p = ((d[20] >> 4) & 0x1) ? p : last_p;
	} else if ((state & 0xF0) == 0xF0) {
		if ((state >> 0) & 0x1) {
			last_v = 0.0f;
			last_i = 0.0f;
			last_p = 0.0f;
		} else {
			if ((state >> 3) & 0x1)
				last_v = 0.0f;
			else
				last_v = ((d[20] >> 6) & 0x1) ? v : last_v;

			if ((state >> 2) & 0x1)
				last_i = 0.0f;
			else
				last_i = ((d[20] >> 5) & 0x1) ? i : last_i;

			if ((state >> 1) & 0x1)
				last_p = 0.0f;
			else
				last_p = ((d[20] >> 4) & 0x1) ? p : last_p;
		}
	}

	/* print to console */
	ESP_LOGD(TAG, "V=%.2fV I=%.4fA P=%.2fW S=%.2fVA PF=%.4f E=%.6fkWh PFcnt=%lu",
		 last_v, last_i, last_p, s, s ? p / s : 0.0f, energy, (unsigned long)pf_cnt);

	last_energy = energy;

	return true;
}

static void hlw8032_rx_task(void *arg)
{
	uint8_t data[24];
	uint32_t rx_count = 0;
	uint32_t err_count = 0;
	bool ret;
	int len;
	struct event_bus_msg msg;

	while (task_running) {
		/* Read data from the UART */
		len = uart_read_bytes(hlw8032_uart, data, sizeof(data), pdMS_TO_TICKS(20));
		if (len == 24) {
			ret = hlw8032_frame_update(data);
			if (ret) {
				rx_count++;
				if (!probed) {
					probed = true;
					xTaskNotifyGive(handle);
				}
			} else {
				err_count++;
			}
		} else if (len) {
			err_count++;
		}

		if (probed && !(rx_count % 10)) {
			ESP_LOGI(TAG, "rx_count: %u, err_count: %u; ", rx_count, err_count);

			msg.type = EVENT_BUS_MEASURE_VOLTAGE_UPDATED;
			msg.param1 = 0;
			msg.param2 = (uint32_t)(last_v * 1000);
			event_bus_send(&msg);
			msg.type = EVENT_BUS_MEASURE_CURRENT_UPDATED;
			msg.param1 = 0;
			msg.param2 = (uint32_t)(last_i * 1000);
			event_bus_send(&msg);
			msg.type = EVENT_BUS_MEASURE_POWER_UPDATED;
			msg.param1 = 0;
			msg.param2 = (uint32_t)(last_p * 1000);
			event_bus_send(&msg);
			msg.type = EVENT_BUS_MEASURE_ENERGY_UPDATED;
			msg.param1 = 0;
			msg.param2 = (uint32_t)(last_energy * 1000);
			event_bus_send(&msg);
		}
	}

	vTaskDelete(NULL);
}

int hlw8032_init(uart_port_t uart_num)
{
	int ret;

	/* Configure parameters of an UART driver, communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = 4800,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_EVEN,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	hlw8032_uart = uart_num;

	/*
	 * UART0 is also the serial port used for log output,
	 * so changing the baud rate here will also change
	 * the baud rate for log output.
	 */
	ESP_LOGW(TAG, "The UART0 baud rate is about to switch to 4800 8E1");
	fflush(stdout);
	ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
	ESP_ERROR_CHECK(uart_param_config(hlw8032_uart, &uart_config));
	ESP_ERROR_CHECK(uart_driver_install(hlw8032_uart, 2400, 0, 0, NULL, 0));

	task_running = true;
	probed = false;
	handle = xTaskGetCurrentTaskHandle();

	ret = xTaskCreate(hlw8032_rx_task, "hlw8032_rx_task", 2048, NULL,
			  tskIDLE_PRIORITY + 10, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);

	ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)); /* 200mS timeout */
	if (!probed) {
		task_running = false;
		/* Wait a moment to make sure the task has exited */
		vTaskDelay(pdMS_TO_TICKS(200));
		ESP_ERROR_CHECK(uart_driver_delete(hlw8032_uart));
		if (CONFIG_CONSOLE_UART_NUM == hlw8032_uart) {
			/* Restore UART */
			uart_config.baud_rate = CONFIG_CONSOLE_UART_BAUDRATE;
			uart_config.data_bits = UART_DATA_8_BITS;
			uart_config.parity = UART_PARITY_DISABLE;
			uart_config.stop_bits = UART_STOP_BITS_1;
			ESP_ERROR_CHECK(uart_param_config(hlw8032_uart, &uart_config));
		}
		ESP_LOGE(TAG, "HLW8032 was not detected");
		return -1;
	}

	/*
	 * Set the log output level to reduce the resources used
	 * by logging (because the baud rate is too low).
	 */
	esp_log_level_set("*", ESP_LOG_WARN);

	return 0;
}
