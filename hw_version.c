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
 *
 * refs: ESP8266_RTOS_SDK/examples/peripherals/adc/main/adc_example_main.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "HW-VERSION";

static int8_t hw_id = -1;

int8_t hw_version_get_id(void)
{
	return hw_id;
}

/*
 * ID             0    1     2    3    4    5    6    7    8    9    10
 * ---------------------------------------------------------------------
 * Voltage(mV)    0   100   200  300  400  500  600  700  800  900  1000
 * Percent(%)     0    10    20   30   40   50   60   70   80   90   100
 * Pull-Up       NC   32K   31K  30K  29K  28K  27K  26K  25K  24K   10K
 * Pull-Down    10K    1K    2K   3K   4K   5K   6K   7K   8K   9K    NC
 */
static const uint16_t lookup_table[11] = { 50, 150, 250, 350, 450, 550, 650, 750, 850, 950, 1000 };

void hw_version_init(void)
{
	adc_config_t adc_config;
	uint16_t adc_data[8];
	uint16_t adv_voltage = 0;
	uint8_t i;

	/*
	 * Depend on menuconfig->Component config->PHY->vdd33_const value
	 * When measuring system voltage(ADC_READ_VDD_MODE), vdd33_const must be set to 255.
	 */
	adc_config.mode = ADC_READ_TOUT_MODE;
	adc_config.clk_div = 8; /* ADC sample collection clock = 80MHz/clk_div = 10MHz */
	ESP_ERROR_CHECK(adc_init(&adc_config));

	/* Sample */
	ESP_ERROR_CHECK(adc_read_fast(adc_data, 8));

	adc_deinit();

	for (i = 0; i < 8; i++)
		adv_voltage += adc_data[i];
	adv_voltage = (adv_voltage * 1000) / 1024 / 8;
	adv_voltage = adv_voltage > 1000 ? 1000 : adv_voltage;
	ESP_LOGI(TAG, "ADC voltage: %umV", adv_voltage);

	for (i = 0; i < 11; i++) {
		if (adv_voltage < lookup_table[i]) {
			hw_id = i;
			break;
		}
	}
	ESP_LOGI(TAG, "HW ID: %u", hw_id);
}
