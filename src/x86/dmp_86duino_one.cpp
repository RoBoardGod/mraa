/*
 * Author: CJ Wu <sayter@dmp.com.tw>
 * Copyright (c) 2016 DM&P Electronic Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "common.h"
#include "x86/dmp_86duino_one.h"

#include "Arduino.h"
#include <signal.h>
#include "86DuinoSDK/irq.h"
#include "86DuinoSDK/i2c.h"
#include "86DuinoSDK/Wire.h"
#include "86DuinoSDK/io.h"
#include "86DuinoSDK/mcm.h"
#include "86DuinoSDK/TimerOne.h"
#include "86DuinoSDK/SPI.h"

#define PLATFORM_NAME "86Duino One"
#define MAX_SIZE 64
#define MAX_MODE_SIZE 8

#define  COM1_TX    (0x9A)
#define  COM1_RX    (0x9B)
#define  COM2_TX    (0x9E)
#define  COM2_RX    (0x9F)
#define  COM3_TX    (0x9C)
#define  COM3_RX    (0x9D)

#define YES    (1)
#define NO     (2)
static int mc_md_inuse[45] = {0};
static int isPwmInit[45] = {0};

void initVariant() __attribute__((weak));
void initVariant() { }

static __attribute__((constructor(101))) void _f_init()
{
    init();
}

mraa_result_t
mraa_dmp_86duino_one_gpio_init_internal_replace(mraa_gpio_context dev, int pin)
{
	dev->owner = 1;
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_gpio_dir_replace(mraa_gpio_context dev, mraa_gpio_dir_t dir)
{
	int pin = dev->pin;
	switch(dir)
	{
		case MRAA_GPIO_OUT:
			pinMode(pin, OUTPUT);
			break;
		case MRAA_GPIO_IN:
			pinMode(pin, INPUT);
			break;
		case MRAA_GPIO_OUT_HIGH:
			pinMode(pin, OUTPUT);
			digitalWrite(pin, HIGH);
			break;
		case MRAA_GPIO_OUT_LOW:
			pinMode(pin, OUTPUT);
			digitalWrite(pin, LOW);
			break;
		default:
			break;
	}
	return MRAA_SUCCESS;
}

int
mraa_dmp_86duino_one_gpio_read_replace(mraa_gpio_context dev)
{
	return digitalRead(dev->pin);
}

mraa_result_t
mraa_dmp_86duino_one_gpio_write_replace(mraa_gpio_context dev, int value)
{
	digitalWrite(dev->pin, value);
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_aio_get_valid_fp(mraa_aio_context dev)
{
	return MRAA_SUCCESS;
}

unsigned int
mraa_dmp_86duino_one_aio_read_replace(mraa_aio_context dev)
{
	return (unsigned int)analogRead(dev->channel);
}

mraa_result_t
mraa_dmp_86duino_one_i2c_init_bus_replace(mraa_i2c_context dev)
{
	Wire.begin();
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_i2c_set_frequency_replace(mraa_i2c_context dev, mraa_i2c_mode_t mode)
{
	switch(mode)
	{
		case MRAA_I2C_STD:
			i2c_SetSpeed(0, I2CMODE_AUTO, 100000L);
			break;
		case MRAA_I2C_FAST:
			i2c_SetSpeed(0, I2CMODE_AUTO, 400000L);
			break;
		case MRAA_I2C_HIGH:
			i2c_SetSpeed(0, I2CMODE_AUTO, 3400000L);
			break;
		default:
			break;
	}
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_i2c_address_replace(mraa_i2c_context dev, uint8_t addr)
{
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_i2c_write_replace(mraa_i2c_context dev, const uint8_t* data, int length)
{
	Wire.beginTransmission(dev->addr);
	for(int i = 0; i < length; ++i) {
		Wire.write(data[i]);
	}
	Wire.endTransmission();
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_i2c_write_byte_replace(mraa_i2c_context dev, const uint8_t data)
{
	Wire.beginTransmission(dev->addr);
	Wire.write(data);
	Wire.endTransmission();
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_i2c_write_byte_data_replace(mraa_i2c_context dev, const uint8_t data, const uint8_t command)
{
	Wire.beginTransmission(dev->addr);
	Wire.write(command);
	Wire.write(data);
	Wire.endTransmission();
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_mraa_i2c_write_word_data_replace(mraa_i2c_context dev, const uint16_t data, const uint8_t command)
{
	uint8_t first_byte = (data & 0xFF00) >> 8;
	uint8_t second_byte = (data & 0x00FF);
	Wire.beginTransmission(dev->addr);
	Wire.write(command);
	Wire.write(first_byte);
	Wire.write(second_byte);
	Wire.endTransmission();
	return MRAA_SUCCESS;
}

int
mraa_dmp_86duino_one_i2c_read_replace(mraa_i2c_context dev, uint8_t* data, int length)
{
	Wire.requestFrom(dev->addr, length);
	for(int i = 0; i < length; ++i) {
		data[i] = Wire.read();
	}
	return length;
}

uint8_t
mraa_dmp_86duino_one_i2c_read_byte_replace(mraa_i2c_context dev)
{
	Wire.requestFrom(dev->addr, 1);
	return Wire.read();
}

uint8_t
mraa_dmp_86duino_one_i2c_read_byte_data_replace(mraa_i2c_context dev, uint8_t command)
{
	mraa_dmp_86duino_one_i2c_write_byte_replace(dev, command);
	Wire.requestFrom(dev->addr, 1);
	return Wire.read();
}

uint16_t
mraa_dmp_86duino_one_i2c_read_word_data_replace(mraa_i2c_context dev, uint8_t command)
{
	mraa_dmp_86duino_one_i2c_write_byte_replace(dev, command);
	Wire.requestFrom(dev->addr, 2);
	uint16_t word_data = (Wire.read() << 8) | Wire.read();
	return word_data;
}

int
mraa_dmp_86duino_one_i2c_read_bytes_data_replace(mraa_i2c_context dev, uint8_t command, uint8_t* data, int length)
{
	mraa_dmp_86duino_one_i2c_write_byte_replace(dev, command);
	Wire.requestFrom(dev->addr, length);
	for(int i = 0; i < length; ++i) {
		data[i] = Wire.read();
	}
	return length;
}

mraa_pwm_context
mraa_dmp_86duino_one_pwm_init_replace(int pin)
{
	mraa_pwm_context pret = (mraa_pwm_context) malloc(sizeof(struct _pwm));
	pret->pin = pin;
	pret->period = 20000;
	pret->duty = 0;
	pret->enable = 0;
	pret->owner = 1;
	return pret;
}

mraa_result_t
mraa_dmp_86duino_one_pwm_period_replace(mraa_pwm_context dev, int period)
{
	if(period < 1) return MRAA_ERROR_INVALID_PARAMETER;
	if(dev->enable != 0)
		Timer1.freePwm(dev->pin, dev->duty/1000, period/1000);
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_pwm_duty_replace(mraa_pwm_context dev, int duty)
{
	if(duty < 0) return MRAA_ERROR_INVALID_PARAMETER;
	if(dev->enable != 0)
		Timer1.freePwm(dev->pin, duty/1000, dev->period/1000);
	return MRAA_SUCCESS;
}

int
mraa_dmp_86duino_one_pwm_read_period_replace(mraa_pwm_context dev)
{
	return dev->period;
}

int
mraa_dmp_86duino_one_pwm_read_duty_replace(mraa_pwm_context dev)
{
	return dev->duty;
}

mraa_result_t
mraa_dmp_86duino_one_pwm_enable_replace(mraa_pwm_context dev, int enable)
{
	dev->enable = enable;
	if (enable == 0)
		Timer1.disablePwm(dev->pin);
	else
		Timer1.freePwm(dev->pin, dev->duty/1000, dev->period/1000);
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_uart_init_pre(int index)
{
	mraa_result_t ret = MRAA_SUCCESS;
	unsigned short crossbar_ioaddr = sb_Read16(0x64)&0xfffe;
	switch(index)
	{
		case 0:
			io_outpb(crossbar_ioaddr + COM1_TX, 0x08);
			io_outpb(crossbar_ioaddr + COM1_RX, 0x08);
			break;
		case 1:
			io_outpb(crossbar_ioaddr + COM2_TX, 0x08);
			io_outpb(crossbar_ioaddr + COM2_RX, 0x08);
			break;
		case 2:
			io_outpb(crossbar_ioaddr + COM3_TX, 0x08);
			io_outpb(crossbar_ioaddr + COM3_RX, 0x08);
			break;
		default:
			ret = MRAA_ERROR_INVALID_PARAMETER;
			break;
	}
	return MRAA_SUCCESS;
}

mraa_spi_context
mraa_dmp_86duino_one_spi_init_raw_replace(mraa_spi_context dev)
{
	SPI.begin();
	dev->mode = 0;
	dev->bpw = 8;
	dev->lsb = 0;
	dev->clock = 4000000;
	return dev;
}

mraa_result_t
mraa_dmp_86duino_one_spi_mode_replace(mraa_spi_context dev, mraa_spi_mode_t mode)
{
	uint8_t spi_mode = 0;
    switch (mode) {
        case MRAA_SPI_MODE0:
            spi_mode = SPI_MODE_0;
			SPI.setDataMode(SPI_MODE0);
            break;
        case MRAA_SPI_MODE1:
            spi_mode = SPI_MODE_1;
			SPI.setDataMode(SPI_MODE1);
            break;
        case MRAA_SPI_MODE2:
            spi_mode = SPI_MODE_2;
			SPI.setDataMode(SPI_MODE2);
            break;
        case MRAA_SPI_MODE3:
            spi_mode = SPI_MODE_3;
			SPI.setDataMode(SPI_MODE3);
            break;
        default:
            spi_mode = SPI_MODE_0;
			SPI.setDataMode(SPI_MODE0);
            break;
    }
	
	dev->mode = spi_mode;
	
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_spi_lsbmode_replace(mraa_spi_context dev, mraa_boolean_t lsb)
{
	uint8_t lsb_mode = (uint8_t) lsb;
	if (lsb == 0)
	{
		dev->lsb = 0;
		SPI.setBitOrder(MSBFIRST);
	}
	else
	{
		dev->lsb = 1;
		SPI.setBitOrder(LSBFIRST);
	}
	
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_spi_frequency_replace(mraa_spi_context dev, int hz)
{
	if(hz > 12207 && hz < 50000001)
	{
		SPI.setClockDivider(50000000/hz);
		dev->clock = hz;
	}
	else
	{
		return MRAA_ERROR_INVALID_PARAMETER;
	}
	
	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_spi_bit_per_word_replace(mraa_spi_context dev, unsigned int bits)
{
	dev->bpw = bits;
	return MRAA_SUCCESS;
}

int
mraa_dmp_86duino_one_spi_write_replace(mraa_spi_context dev, uint8_t data)
{
	SPI.setSS(LOW);
	uint8_t recv = SPI.transfer(data);
	SPI.setSS(HIGH);
	return recv;
}

uint16_t
mraa_dmp_86duino_one_spi_write_word_replace(mraa_spi_context dev, uint16_t data)
{
	SPI.setSS(LOW);
	uint16_t recv = SPI.transfer16(data);
	SPI.setSS(HIGH);
	return recv;
}

mraa_result_t
mraa_dmp_86duino_one_spi_transfer_buf_replace(mraa_spi_context dev, uint8_t* data, uint8_t* rxbuf, int length)
{
	for(int i = 0; i < length; i++)
	{
		SPI.setSS(LOW);
		*(rxbuf++) = SPI.transfer(*(data++));
		SPI.setSS(HIGH);
	}

	return MRAA_SUCCESS;
}

mraa_result_t
mraa_dmp_86duino_one_spi_transfer_buf_word_replace(mraa_spi_context dev, uint16_t* data, uint16_t* rxbuf, int length)
{
	for(int i = 0; i < length; i++)
	{
		SPI.setSS(LOW);
		*(rxbuf++) = SPI.transfer16(*(data++));
		SPI.setSS(HIGH);
	}

	return MRAA_SUCCESS;
}

mraa_board_t*
mraa_dmp_86duino_one()
{
	interrupt_init();
    initVariant();
	
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->platform_name = PLATFORM_NAME;

    b->phy_pin_count = 59;
    b->gpio_count = 45;
    b->aio_count = 7;
	b->no_bus_mux = 1;

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        goto error;
    }
	b->adv_func->gpio_init_internal_replace = &mraa_dmp_86duino_one_gpio_init_internal_replace;
	b->adv_func->gpio_dir_replace = &mraa_dmp_86duino_one_gpio_dir_replace;
	b->adv_func->gpio_read_replace = &mraa_dmp_86duino_one_gpio_read_replace;
	b->adv_func->gpio_write_replace = &mraa_dmp_86duino_one_gpio_write_replace;
	b->adv_func->aio_get_valid_fp = &mraa_dmp_86duino_one_aio_get_valid_fp;
	b->adv_func->aio_read_replace = &mraa_dmp_86duino_one_aio_read_replace;
	b->adv_func->i2c_init_bus_replace = &mraa_dmp_86duino_one_i2c_init_bus_replace;
	b->adv_func->i2c_set_frequency_replace = &mraa_dmp_86duino_one_i2c_set_frequency_replace;
	b->adv_func->i2c_address_replace = &mraa_dmp_86duino_one_i2c_address_replace;
	b->adv_func->i2c_read_replace = &mraa_dmp_86duino_one_i2c_read_replace;
	b->adv_func->i2c_read_byte_replace = &mraa_dmp_86duino_one_i2c_read_byte_replace;
	b->adv_func->i2c_read_byte_data_replace = &mraa_dmp_86duino_one_i2c_read_byte_data_replace;
	b->adv_func->i2c_read_word_data_replace = &mraa_dmp_86duino_one_i2c_read_word_data_replace;
	b->adv_func->i2c_read_bytes_data_replace = &mraa_dmp_86duino_one_i2c_read_bytes_data_replace;
	b->adv_func->i2c_write_replace = &mraa_dmp_86duino_one_i2c_write_replace;
	b->adv_func->i2c_write_byte_replace = &mraa_dmp_86duino_one_i2c_write_byte_replace;
	b->adv_func->i2c_write_byte_data_replace = &mraa_dmp_86duino_one_i2c_write_byte_data_replace;
	b->adv_func->i2c_write_word_data_replace = &mraa_dmp_86duino_one_mraa_i2c_write_word_data_replace;
	b->adv_func->pwm_init_replace = &mraa_dmp_86duino_one_pwm_init_replace;
	b->adv_func->pwm_period_replace = &mraa_dmp_86duino_one_pwm_period_replace;
	b->adv_func->pwm_duty_replace = &mraa_dmp_86duino_one_pwm_duty_replace;
	b->adv_func->pwm_read_period_replace = &mraa_dmp_86duino_one_pwm_read_period_replace;
	b->adv_func->pwm_read_duty_replace = &mraa_dmp_86duino_one_pwm_read_duty_replace;
	b->adv_func->pwm_enable_replace = &mraa_dmp_86duino_one_pwm_enable_replace;
	b->adv_func->uart_init_pre = &mraa_dmp_86duino_one_uart_init_pre;
	b->adv_func->spi_init_raw_replace = &mraa_dmp_86duino_one_spi_init_raw_replace;
	b->adv_func->spi_mode_replace = &mraa_dmp_86duino_one_spi_mode_replace;
	b->adv_func->spi_lsbmode_replace = &mraa_dmp_86duino_one_spi_lsbmode_replace;
	b->adv_func->spi_frequency_replace = &mraa_dmp_86duino_one_spi_frequency_replace;
	b->adv_func->spi_bit_per_word_replace = &mraa_dmp_86duino_one_spi_bit_per_word_replace;
	b->adv_func->spi_write_replace = &mraa_dmp_86duino_one_spi_write_replace;
	b->adv_func->spi_write_word_replace = &mraa_dmp_86duino_one_spi_write_word_replace;
	b->adv_func->spi_transfer_buf_replace = &mraa_dmp_86duino_one_spi_transfer_buf_replace;
	b->adv_func->spi_transfer_buf_word_replace = &mraa_dmp_86duino_one_spi_transfer_buf_word_replace;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * MRAA_DMP_86DUINO_ONE_PINCOUNT);
    if (b->pins == NULL) {
        free(b->adv_func);
        goto error;
    }

    b->adc_raw = 11;
    b->adc_supported = 10;
    b->pwm_default_period = 20000;
    b->pwm_max_period = 1000000;
    b->pwm_min_period = 1;

    strncpy(b->pins[0].name, "IO0", 8);
    b->pins[0].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[0].gpio.pinmap = 0;//11;
    b->pins[0].gpio.parent_id = 0;
    b->pins[0].gpio.mux_total = 0;
    b->pins[0].uart.pinmap = 0;
    b->pins[0].uart.parent_id = 0;
    b->pins[0].uart.mux_total = 0;

    strncpy(b->pins[1].name, "IO1", 8);
    b->pins[1].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[1].gpio.pinmap = 1;//10;
    b->pins[1].gpio.parent_id = 0;
    b->pins[1].gpio.mux_total = 0;
    b->pins[1].uart.pinmap = 0;
    b->pins[1].uart.parent_id = 0;
    b->pins[1].uart.mux_total = 0;

    strncpy(b->pins[2].name, "IO2", 8);
    b->pins[2].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0 };
    b->pins[2].gpio.pinmap = 2;//9;
    b->pins[2].gpio.parent_id = 0;
    b->pins[2].gpio.mux_total = 0;

    strncpy(b->pins[3].name, "IO3", 8);
    b->pins[3].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0 };
    b->pins[3].gpio.pinmap = 3;//23;
    b->pins[3].gpio.parent_id = 0;
    b->pins[3].gpio.mux_total = 0;
    b->pins[3].pwm.pinmap = 0;
    b->pins[3].pwm.parent_id = 3;
    b->pins[3].pwm.mux_total = 0;

    strncpy(b->pins[4].name, "IO4", 8);
    b->pins[4].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0 };
    b->pins[4].gpio.pinmap = 4;//;
    b->pins[4].gpio.parent_id = 0;
    b->pins[4].gpio.mux_total = 0;

    strncpy(b->pins[5].name, "IO5", 8);
    b->pins[5].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[5].gpio.pinmap = 5;//20;
    b->pins[5].gpio.parent_id = 0;
    b->pins[5].gpio.mux_total = 0;
    b->pins[5].pwm.pinmap = 2;
    b->pins[5].pwm.parent_id = 0;
    b->pins[5].pwm.mux_total = 0;

    strncpy(b->pins[6].name, "IO6", 8);
    b->pins[6].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[6].gpio.pinmap = 6;//19;
    b->pins[6].gpio.parent_id = 0;
    b->pins[6].gpio.mux_total = 0;
    b->pins[6].pwm.pinmap = 1;
    b->pins[6].pwm.parent_id = 0;
    b->pins[6].pwm.mux_total = 0;

    strncpy(b->pins[7].name, "IO7", 8);
    b->pins[7].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0 };
    b->pins[7].gpio.pinmap = 7;//35;
    b->pins[7].gpio.parent_id = 0;
    b->pins[7].gpio.mux_total = 0;

    strncpy(b->pins[8].name, "IO8", 8);
    b->pins[8].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0 };
    b->pins[8].gpio.pinmap = 8;//33;
    b->pins[8].gpio.parent_id = 0;
    b->pins[8].gpio.mux_total = 0;

    strncpy(b->pins[9].name, "IO9", 8);
    b->pins[9].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[9].gpio.pinmap = 9;//17;
    b->pins[9].gpio.parent_id = 0;
    b->pins[9].gpio.mux_total = 0;
    b->pins[9].pwm.pinmap = 0;
    b->pins[9].pwm.parent_id = 0;
    b->pins[9].pwm.mux_total = 0;

    strncpy(b->pins[10].name, "IO10", 8);
    b->pins[10].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[10].gpio.pinmap = 10;//28;
    b->pins[10].gpio.parent_id = 0;
    b->pins[10].gpio.mux_total = 0;
	b->pins[10].pwm.pinmap = 2;
    b->pins[10].pwm.parent_id = 1;
    b->pins[10].pwm.mux_total = 0;

    strncpy(b->pins[11].name, "IO11", 8);
    b->pins[11].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[11].gpio.pinmap = 11;//27;
    b->pins[11].gpio.parent_id = 0;
    b->pins[11].gpio.mux_total = 0;
	b->pins[11].pwm.pinmap = 1;
    b->pins[11].pwm.parent_id = 1;
    b->pins[11].pwm.mux_total = 0;

    strncpy(b->pins[12].name, "IO12", 8);
    b->pins[12].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[12].gpio.pinmap = 12;//32;
    b->pins[12].gpio.parent_id = 0;
    b->pins[12].gpio.mux_total = 0;

    strncpy(b->pins[13].name, "IO13", 8);
    b->pins[13].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[13].gpio.pinmap = 13;//25;
    b->pins[13].gpio.parent_id = 0;
    b->pins[13].gpio.mux_total = 0;
	b->pins[13].pwm.pinmap = 0;
    b->pins[13].pwm.parent_id = 1;
    b->pins[13].pwm.mux_total = 0;
	
	strncpy(b->pins[12].name, "IO14", 8);
    b->pins[14].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[14].gpio.pinmap = 14;//12;
    b->pins[14].gpio.parent_id = 0;
    b->pins[14].gpio.mux_total = 0;
	b->pins[14].uart.pinmap = 2;
    b->pins[14].uart.parent_id = 0;
    b->pins[14].uart.mux_total = 0;
	
	strncpy(b->pins[12].name, "IO15", 8);
    b->pins[15].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[15].gpio.pinmap = 15;//13;
    b->pins[15].gpio.parent_id = 0;
    b->pins[15].gpio.mux_total = 0;
	b->pins[15].uart.pinmap = 2;
    b->pins[15].uart.parent_id = 0;
    b->pins[15].uart.mux_total = 0;
	
	strncpy(b->pins[12].name, "IO16", 8);
    b->pins[16].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[16].gpio.pinmap = 16;//14;
    b->pins[16].gpio.parent_id = 0;
    b->pins[16].gpio.mux_total = 0;
	b->pins[16].uart.pinmap = 1;
    b->pins[16].uart.parent_id = 0;
    b->pins[16].uart.mux_total = 0;
	
	strncpy(b->pins[12].name, "IO17", 8);
    b->pins[17].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[17].gpio.pinmap = 17;//15;
    b->pins[17].gpio.parent_id = 0;
    b->pins[17].gpio.mux_total = 0;
	b->pins[17].uart.pinmap = 1;
    b->pins[17].uart.parent_id = 0;
    b->pins[17].uart.mux_total = 0;
	
	strncpy(b->pins[18].name, "IO18", 8);
    b->pins[18].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[18].gpio.pinmap = 18;//24;
    b->pins[18].gpio.parent_id = 0;
    b->pins[18].gpio.mux_total = 0;
	
	strncpy(b->pins[19].name, "IO19", 8);
    b->pins[19].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[19].gpio.pinmap = 19;//26;
    b->pins[19].gpio.parent_id = 0;
    b->pins[19].gpio.mux_total = 0;
	
	strncpy(b->pins[20].name, "IO20", 8);
    b->pins[20].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[20].gpio.pinmap = 20;//29;
    b->pins[20].gpio.parent_id = 0;
    b->pins[20].gpio.mux_total = 0;
	
	strncpy(b->pins[21].name, "IO21", 8);
    b->pins[21].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[21].gpio.pinmap = 21;//47;
    b->pins[21].gpio.parent_id = 0;
    b->pins[21].gpio.mux_total = 0;
	
	strncpy(b->pins[22].name, "IO22", 8);
    b->pins[22].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[22].gpio.pinmap = 22;//46;
    b->pins[22].gpio.parent_id = 0;
    b->pins[22].gpio.mux_total = 0;
	
	strncpy(b->pins[23].name, "IO23", 8);
    b->pins[23].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[23].gpio.pinmap = 23;//45;
    b->pins[23].gpio.parent_id = 0;
    b->pins[23].gpio.mux_total = 0;
	
	strncpy(b->pins[24].name, "IO24", 8);
    b->pins[24].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[24].gpio.pinmap = 24;//44;
    b->pins[24].gpio.parent_id = 0;
    b->pins[24].gpio.mux_total = 0;
	
	strncpy(b->pins[25].name, "IO25", 8);
    b->pins[25].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[25].gpio.pinmap = 25;//43;
    b->pins[25].gpio.parent_id = 0;
    b->pins[25].gpio.mux_total = 0;
	
	strncpy(b->pins[26].name, "IO26", 8);
    b->pins[26].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[26].gpio.pinmap = 26;//42;
    b->pins[26].gpio.parent_id = 0;
    b->pins[26].gpio.mux_total = 0;
	
	strncpy(b->pins[27].name, "IO27", 8);
    b->pins[27].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[27].gpio.pinmap = 27;//41;
    b->pins[27].gpio.parent_id = 0;
    b->pins[27].gpio.mux_total = 0;
	
	strncpy(b->pins[28].name, "IO28", 8);
    b->pins[28].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[28].gpio.pinmap = 28;//40;
    b->pins[28].gpio.parent_id = 0;
    b->pins[28].gpio.mux_total = 0;
	
	strncpy(b->pins[29].name, "IO29", 8);
    b->pins[29].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[29].gpio.pinmap = 29;//1;
    b->pins[29].gpio.parent_id = 0;
    b->pins[29].gpio.mux_total = 0;
	b->pins[29].pwm.pinmap = 0;
    b->pins[29].pwm.parent_id = 2;
    b->pins[29].pwm.mux_total = 0;
	
	strncpy(b->pins[30].name, "IO30", 8);
    b->pins[30].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[30].gpio.pinmap = 30;//3;
    b->pins[30].gpio.parent_id = 0;
    b->pins[30].gpio.mux_total = 0;
	b->pins[30].pwm.pinmap = 1;
    b->pins[30].pwm.parent_id = 2;
    b->pins[30].pwm.mux_total = 0;
	
	strncpy(b->pins[31].name, "IO31", 8);
    b->pins[31].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[31].gpio.pinmap = 31;//4;
    b->pins[31].gpio.parent_id = 0;
    b->pins[31].gpio.mux_total = 0;
	b->pins[31].pwm.pinmap = 2;
    b->pins[31].pwm.parent_id = 2;
    b->pins[31].pwm.mux_total = 0;
	
	strncpy(b->pins[32].name, "IO32", 8);
    b->pins[32].capabilites = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[32].gpio.pinmap = 32;//31;
    b->pins[32].gpio.parent_id = 0;
    b->pins[32].gpio.mux_total = 0;
	b->pins[32].pwm.pinmap = 1;
    b->pins[32].pwm.parent_id = 3;
    b->pins[32].pwm.mux_total = 0;
	
	strncpy(b->pins[33].name, "IO33", 8);
    b->pins[33].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[33].gpio.pinmap = 33;//0;
    b->pins[33].gpio.parent_id = 0;
    b->pins[33].gpio.mux_total = 0;
	
	strncpy(b->pins[34].name, "IO34", 8);
    b->pins[34].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[34].gpio.pinmap = 34;//2;
    b->pins[34].gpio.parent_id = 0;
    b->pins[34].gpio.mux_total = 0;
	
	strncpy(b->pins[35].name, "IO35", 8);
    b->pins[35].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[35].gpio.pinmap = 35;//5;
    b->pins[35].gpio.parent_id = 0;
    b->pins[35].gpio.mux_total = 0;
	
	strncpy(b->pins[36].name, "IO36", 8);
    b->pins[36].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[36].gpio.pinmap = 36;//22;
    b->pins[36].gpio.parent_id = 0;
    b->pins[36].gpio.mux_total = 0;
	
	strncpy(b->pins[37].name, "IO37", 8);
    b->pins[37].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[37].gpio.pinmap = 37;//30;
    b->pins[37].gpio.parent_id = 0;
    b->pins[37].gpio.mux_total = 0;
	
	strncpy(b->pins[38].name, "IO38", 8);
    b->pins[38].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[38].gpio.pinmap = 38;//6;
    b->pins[38].gpio.parent_id = 0;
    b->pins[38].gpio.mux_total = 0;
	
	strncpy(b->pins[39].name, "IO39", 8);
    b->pins[39].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[39].gpio.pinmap = 39;//38;
    b->pins[39].gpio.parent_id = 0;
    b->pins[39].gpio.mux_total = 0;
	
	strncpy(b->pins[40].name, "IO40", 8);
    b->pins[40].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[40].gpio.pinmap = 40;//36;
    b->pins[40].gpio.parent_id = 0;
    b->pins[40].gpio.mux_total = 0;
	
	strncpy(b->pins[41].name, "IO41", 8);
    b->pins[41].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[41].gpio.pinmap = 41;//34;
    b->pins[41].gpio.parent_id = 0;
    b->pins[41].gpio.mux_total = 0;
	
	strncpy(b->pins[42].name, "IO42", 8);
    b->pins[42].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[42].gpio.pinmap = 42;//16;
    b->pins[42].gpio.parent_id = 0;
    b->pins[42].gpio.mux_total = 0;
	
	strncpy(b->pins[43].name, "IO43", 8);
    b->pins[43].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[43].gpio.pinmap = 43;//18;
    b->pins[43].gpio.parent_id = 0;
    b->pins[43].gpio.mux_total = 0;
	
	strncpy(b->pins[44].name, "IO44", 8);
    b->pins[44].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[44].gpio.pinmap = 44;//21;
    b->pins[44].gpio.parent_id = 0;
    b->pins[44].gpio.mux_total = 0;

    strncpy(b->pins[45].name, "A0", 8);
    b->pins[45].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[45].aio.pinmap = 0;
    b->pins[45].aio.mux_total = 0;
    b->pins[45].gpio.pinmap = 45;
    b->pins[45].gpio.mux_total = 0;

    strncpy(b->pins[46].name, "A1", 8);
    b->pins[46].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[46].aio.pinmap = 1;
    b->pins[46].aio.mux_total = 0;
    b->pins[46].gpio.pinmap = 46;
    b->pins[46].gpio.mux_total = 0;

    strncpy(b->pins[47].name, "A2", 8);
    b->pins[47].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[47].aio.pinmap = 2;
    b->pins[47].aio.mux_total = 0;
    b->pins[47].gpio.pinmap = 47;
    b->pins[47].gpio.mux_total = 0;

    strncpy(b->pins[48].name, "A3", 8);
    b->pins[48].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[48].aio.pinmap = 3;
    b->pins[48].aio.mux_total = 0;
    b->pins[48].gpio.pinmap = 48;
    b->pins[48].gpio.mux_total = 0;

    strncpy(b->pins[49].name, "A4", 8);
    b->pins[49].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[49].aio.pinmap = 4;
    b->pins[49].aio.mux_total = 0;
    b->pins[49].gpio.pinmap = 49;
    b->pins[49].gpio.mux_total = 0;

    strncpy(b->pins[50].name, "A5", 8);
    b->pins[50].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[50].aio.pinmap = 5;
    b->pins[50].aio.mux_total = 0;
    b->pins[50].gpio.pinmap = 50;
    b->pins[50].gpio.mux_total = 0;
	
	strncpy(b->pins[51].name, "A6", 8);
    b->pins[51].capabilites = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[51].aio.pinmap = 6;
    b->pins[51].aio.mux_total = 0;
    b->pins[51].gpio.pinmap = 51;
    b->pins[51].gpio.mux_total = 0;
	
	strncpy(b->pins[52].name, "SDA", 8);
    b->pins[52].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 };
	b->pins[52].i2c.pinmap = 52;
	b->pins[52].i2c.parent_id = 0;
	b->pins[52].i2c.mux_total = 0;
	
	strncpy(b->pins[53].name, "SCL", 8);
    b->pins[53].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 };
	b->pins[53].i2c.pinmap = 53;
	b->pins[53].i2c.parent_id = 0;
	b->pins[53].i2c.mux_total = 0;
	
	strncpy(b->pins[54].name, "SS", 8);
	b->pins[54].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
	b->pins[54].spi.pinmap = 54;
	b->pins[54].spi.parent_id = 0;
	b->pins[54].spi.mux_total = 0;
	
	strncpy(b->pins[55].name, "MOSI", 8);
	b->pins[55].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
	b->pins[55].spi.pinmap = 55;
	b->pins[55].spi.parent_id = 0;
	b->pins[55].spi.mux_total = 0;
	
	strncpy(b->pins[56].name, "MISO", 8);
	b->pins[56].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
	b->pins[56].spi.pinmap = 56;
	b->pins[56].spi.parent_id = 0;
	b->pins[56].spi.mux_total = 0;
	
	strncpy(b->pins[57].name, "SCK", 8);
	b->pins[57].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
	b->pins[57].spi.pinmap = 57;
	b->pins[57].spi.parent_id = 0;
	b->pins[57].spi.mux_total = 0;
	
	strncpy(b->pins[58].name, "CS", 8);
	b->pins[58].capabilites = (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 };
	b->pins[58].spi.pinmap = 58;
	b->pins[58].spi.parent_id = 0;
	b->pins[58].spi.mux_total = 0;

    // BUS DEFINITIONS
    b->i2c_bus_count = 1;
    b->def_i2c_bus = 0;

    b->i2c_bus[0].bus_id = 0;
    b->i2c_bus[0].sda = 52;
    b->i2c_bus[0].scl = 53;

    b->spi_bus_count = 1;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 0;
    b->spi_bus[0].slave_s = 54;
    b->spi_bus[0].cs = 58;
    b->spi_bus[0].mosi = 55;
    b->spi_bus[0].miso = 56;
    b->spi_bus[0].sclk = 57;

    b->uart_dev_count = 3;
    b->def_uart_dev = 0;
    b->uart_dev[0].rx = 0;
    b->uart_dev[0].tx = 1;
    b->uart_dev[0].device_path = "/dev/ttyS0";
	b->uart_dev[1].rx = 17;
    b->uart_dev[1].tx = 16;
    b->uart_dev[1].device_path = "/dev/ttyS1";
	b->uart_dev[2].rx = 15;
    b->uart_dev[2].tx = 14;
    b->uart_dev[2].device_path = "/dev/ttyS2";

    return b;
error:
    syslog(LOG_CRIT, "86Duino failed to initialise");
    free(b);
    return NULL;
}
