#ifndef INIT_STWIN_SENSORS_H
#define INIT_STWIN_SENSORS_H

/*---------- GLOBAL ----------*/
#include "main.h"

int init_sensors(UART_HandleTypeDef *UART_bus, I2C_HandleTypeDef *stts751_bus, I2C_HandleTypeDef *lps22hh_bus,
		I2C_HandleTypeDef *hts221_bus, SPI_HandleTypeDef *ism330_bus);

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

/*---------- HTS221 ----------*/
#include "hts221_reg.h"

int16_t hts221_data_raw_humidity;
int16_t hts221_data_raw_temperature;
float hts221_humidity_perc;
float hts221_temperature_degC;
uint8_t hts221_whoamI;
stmdev_ctx_t hts221_dev_ctx;
hts221_reg_t hts221_reg;

typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;
lin_t hts221_lin_hum;
lin_t hts221_lin_temp;

float linear_interpolation(lin_t *lin, int16_t x);
int init_hts221(I2C_HandleTypeDef *hts221_bus);

/*---------- ISM330 ----------*/
#include "ism330dhcx_reg.h"

int16_t ism330_data_raw_acceleration[3];
int16_t ism330_data_raw_angular_rate[3];
int16_t ism330_data_raw_temperature;
float ism330_acceleration_mg[3];
float ism330_angular_rate_mdps[3];
float ism330_temperature_degC;
uint8_t ism330_whoamI;
uint8_t ism330_rst;
stmdev_ctx_t ism330_dev_ctx;
uint8_t ism330_reg;

int init_ism330(SPI_HandleTypeDef *ism330_bus);

#endif
