#include "init_stwin_sensors.h"

/*---------- GLOBAL ----------*/
int init_sensors(UART_HandleTypeDef *UART_bus, I2C_HandleTypeDef *stts751_bus, I2C_HandleTypeDef *lps22hh_bus, I2C_HandleTypeDef *hts221_bus)
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
	/*
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
	*/

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
	  return -2;
	}

	return 0;
}

/*---------- STTS 751 ----------*/
int init_stts751(I2C_HandleTypeDef *stts751_bus)
{
	stts751_dev_ctx.write_reg = platform_write;
	stts751_dev_ctx.read_reg = platform_read;
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

/*---------- LPS22HH ----------*/
int init_lps22hh(I2C_HandleTypeDef *lps22hh_bus)
{
	lps22hh_dev_ctx.write_reg = platform_write;
	lps22hh_dev_ctx.read_reg = platform_read;
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

/*---------- HTS221 ----------*/
int init_hts221(I2C_HandleTypeDef *hts221_bus)
{
	hts221_dev_ctx.write_reg = platform_write;
	hts221_dev_ctx.read_reg = platform_read;
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
