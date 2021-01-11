/**
  ******************************************************************************
  * @file           : init_stwin_sensors.c
  * @brief          : Sensors functions for stwin board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 enzopicas.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "init_stwin_sensors.h"

/*---------- GLOBAL ----------*/
int init_sensors(UART_HandleTypeDef *UART_bus, I2C_HandleTypeDef *stts751_bus, I2C_HandleTypeDef *lps22hh_bus,
		I2C_HandleTypeDef *hts221_bus, SPI_HandleTypeDef *ism330_bus)
{
	uint16_t uart_buf_len;
	uint8_t uart_buf[500] = "";

	/*-------- START --------*/
	uart_buf_len = sprintf(uart_buf, "---- Demarrage de l'UART ----\r\n");
	HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);

	/*-------- STTS751 --------*/
	if (init_stts751(stts751_bus) == 0)
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init STTS751 OK\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	}
	else
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init STTS751 Failure\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	  return -1;
	}

	/*-------- LPS22HH --------*/
	if (init_lps22hh(lps22hh_bus) == 0)
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init LPS22HH OK\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	}
	else
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init LPS22HH Failure\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	  return -2;
	}

	/*-------- HTS221 --------*/

	if (init_hts221(hts221_bus) == 0)
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init HTS221 OK\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	}
	else
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init HTS221 Failure\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	  return -3;
	}


	/*-------- ISM330 --------*/
	/*
	if (init_ism330(ism330_bus) == 0)
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init ISM330 OK\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	}
	else
	{
	  uart_buf_len = sprintf(uart_buf, "---- Init ISM330 Failure\r\n");
	  HAL_UART_Transmit(UART_bus, uart_buf, uart_buf_len, 100);
	  return -4;
	}
	*/

	return 0;
}

/*---------- STTS 751 ----------*/
int init_stts751(I2C_HandleTypeDef *stts751_bus)
{
	stts751_id_t stts751_whoamI;

	stts751_dev_ctx.write_reg = platform_write_stts751;
	stts751_dev_ctx.read_reg = platform_read_stts751;
	stts751_dev_ctx.handle = stts751_bus;

	stts751_device_id_get(&stts751_dev_ctx, &stts751_whoamI);
	if ( (stts751_whoamI.product_id != STTS751_ID_0xxxx) ||
	     (stts751_whoamI.manufacturer_id != STTS751_ID_MAN) ||
	     (stts751_whoamI.revision_id != STTS751_REV) )
	{
	   return -1;
	}

	/* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
	float temperature_high_limit = 49.5f;
	stts751_high_temperature_threshold_set(&stts751_dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

	float temperature_low_limit = -4.5f;
	stts751_low_temperature_threshold_set(&stts751_dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

	stts751_pin_event_route_set(&stts751_dev_ctx,  PROPERTY_ENABLE);

	/* Set Output Data Rate */
	stts751_temp_data_rate_set(&stts751_dev_ctx, STTS751_TEMP_ODR_8Hz);

	/* Set Resolution */
	stts751_resolution_set(&stts751_dev_ctx, STTS751_12bit);

	return 0;
}

int32_t platform_write_stts751(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, STTS751_0xxxx_ADD_7K5, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

int32_t platform_read_stts751(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, STTS751_0xxxx_ADD_7K5, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

/*---------- LPS22HH ----------*/
int init_lps22hh(I2C_HandleTypeDef *lps22hh_bus)
{
	uint8_t lps22hh_whoamI;
	uint8_t lps22hh_rst;

	lps22hh_dev_ctx.write_reg = platform_write_lps22hh;
	lps22hh_dev_ctx.read_reg = platform_read_lps22hh;
	lps22hh_dev_ctx.handle = lps22hh_bus;

	lps22hh_device_id_get(&lps22hh_dev_ctx, &lps22hh_whoamI);
	if (lps22hh_whoamI != LPS22HH_ID)
	{
		return -1;
	}

	lps22hh_reset_set(&lps22hh_dev_ctx, PROPERTY_ENABLE);
	do {
	  lps22hh_reset_get(&lps22hh_dev_ctx, &lps22hh_rst);
	} while (lps22hh_rst);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&lps22hh_dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lps22hh_data_rate_set(&lps22hh_dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

	return 0;
}

int32_t platform_write_lps22hh(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

int32_t platform_read_lps22hh(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

/*---------- HTS221 ----------*/
int init_hts221(I2C_HandleTypeDef *hts221_bus)
{
	uint8_t hts221_whoamI;

	hts221_dev_ctx.write_reg = platform_write_hts221;
	hts221_dev_ctx.read_reg = platform_read_hts221;
	hts221_dev_ctx.handle = hts221_bus;

	hts221_whoamI = 0;
	hts221_device_id_get(&hts221_dev_ctx, &hts221_whoamI);
	if (hts221_whoamI != HTS221_ID)
	{
		return -1;
	}

	hts221_hum_adc_point_0_get(&hts221_dev_ctx, &hts221_lin_hum.x0);
	hts221_hum_rh_point_0_get(&hts221_dev_ctx, &hts221_lin_hum.y0);
	hts221_hum_adc_point_1_get(&hts221_dev_ctx, &hts221_lin_hum.x1);
	hts221_hum_rh_point_1_get(&hts221_dev_ctx, &hts221_lin_hum.y1);
	/* Read temperature calibration coefficient */
	hts221_temp_adc_point_0_get(&hts221_dev_ctx, &hts221_lin_temp.x0);
	hts221_temp_deg_point_0_get(&hts221_dev_ctx, &hts221_lin_temp.y0);
	hts221_temp_adc_point_1_get(&hts221_dev_ctx, &hts221_lin_temp.x1);
	hts221_temp_deg_point_1_get(&hts221_dev_ctx, &hts221_lin_temp.y1);
	/* Enable Block Data Update */
	hts221_block_data_update_set(&hts221_dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	hts221_data_rate_set(&hts221_dev_ctx, HTS221_ODR_7Hz);
	/* Device power on */
	hts221_power_on_set(&hts221_dev_ctx, PROPERTY_ENABLE);

	return 0;
}

float linear_interpolation(lin_t *lin, int16_t x)
{
	return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
	           (lin->x0 * lin->y1))) / (lin->x1 - lin->x0);
}

int32_t platform_write_hts221(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*---------- ISM330 ----------*/
/*
int init_ism330(SPI_HandleTypeDef *ism330_bus)
{
	ism330_dev_ctx.write_reg = platform_write;
	ism330_dev_ctx.read_reg = platform_read;
	ism330_dev_ctx.handle = &ism330_bus;

	ism330dhcx_device_id_get(&ism330_dev_ctx, &ism330_whoamI);
	if (ism330_whoamI != ISM330DHCX_ID)
	{
		return -1;
	}

	ism330dhcx_reset_set(&ism330_dev_ctx, PROPERTY_ENABLE);
	do
	{
		ism330dhcx_reset_get(&ism330_dev_ctx, &ism330_rst);
	} while (ism330_rst);

	//* Enable Block Data Update *
	ism330dhcx_block_data_update_set(&ism330_dev_ctx, PROPERTY_ENABLE);
	//* Set Output Data Rate
	ism330dhcx_xl_data_rate_set(&ism330_dev_ctx, ISM330DHCX_XL_ODR_12Hz5);
	ism330dhcx_gy_data_rate_set(&ism330_dev_ctx, ISM330DHCX_GY_ODR_12Hz5);
	//* Set full scale *
	ism330dhcx_xl_full_scale_set(&ism330_dev_ctx, ISM330DHCX_2g);
	ism330dhcx_gy_full_scale_set(&ism330_dev_ctx, ISM330DHCX_2000dps);
	//* Configure filtering chain(No aux interface) *
	ism330dhcx_xl_hp_path_on_out_set(&ism330_dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
	ism330dhcx_xl_filter_lp2_set(&ism330_dev_ctx, PROPERTY_ENABLE);

	return 0;
}
*/
