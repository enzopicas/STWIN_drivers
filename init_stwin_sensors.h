#ifndef INIT_STWIN_SENSORS_H
#define INIT_STWIN_SENSORS_H

/*---------- GLOBAL ----------*/
#include "main.h"

/*---------- STTS 751 ----------*/
#include "stts751_reg.h"

typedef union {
	int16_t i16bit;
	uint8_t u8bit[2];
} axis1bit16_t;

axis1bit16_t data_raw_temperature;
float temperature_degC;
stts751_id_t whoamI;
stmdev_ctx_t dev_ctx;

int init_stts751(I2C_HandleTypeDef *stts751_bus);

#endif
