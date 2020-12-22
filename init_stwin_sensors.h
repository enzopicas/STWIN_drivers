#ifndef INIT_STWIN_SENSORS_H
#define INIT_STWIN_SENSORS_H

/*---------- GLOBAL ----------*/
#include "main.h"

int init_sensors(UART_HandleTypeDef *UART_bus, I2C_HandleTypeDef *stts751_bus, I2C_HandleTypeDef *lps22hh_bus);

/*---------- STTS 751 ----------*/
#include "stts751_reg.h"

typedef union {
	int16_t i16bit;
	uint8_t u8bit[2];
} axis1bit16_t;

axis1bit16_t stts751_data_raw_temperature;
float stts751_temperature_degC;
stts751_id_t stts751_whoamI;
stmdev_ctx_t stts751_dev_ctx;

int init_stts751(I2C_HandleTypeDef *stts751_bus);

/*---------- LPS22HH ----------*/
#include "lps22hh_reg.h"

uint32_t lps22hh_data_raw_pressure;
int16_t lps22hh_data_raw_temperature;
float lps22hh_pressure_hPa;
float lps22hh_temperature_degC;
uint8_t lps22hh_whoamI;
uint8_t lps22hh_rst;
stmdev_ctx_t lps22hh_dev_ctx;
lps22hh_reg_t lps22hh_reg;

int init_lps22hh(I2C_HandleTypeDef *lps22hh_bus);

#endif
