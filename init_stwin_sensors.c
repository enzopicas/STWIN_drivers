#include "init_stwin_sensors.h"

/*---------- GLOBAL ----------*/

/*---------- STTS 751 ----------*/
int init_stts751(I2C_HandleTypeDef *stts751_bus)
{
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = stts751_bus;

	stts751_device_id_get(&dev_ctx, &whoamI);
	if ( (whoamI.product_id != STTS751_ID_0xxxx) ||
	     (whoamI.manufacturer_id != STTS751_ID_MAN) ||
	     (whoamI.revision_id != STTS751_REV) )
	{
	   return -1;
	}

	/* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
	float temperature_high_limit = 49.5f;
	stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

	float temperature_low_limit = -4.5f;
	stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

	stts751_pin_event_route_set(&dev_ctx,  PROPERTY_ENABLE);

	/* Set Output Data Rate */
	stts751_temp_data_rate_set(&dev_ctx, STTS751_TEMP_ODR_8Hz);

	/* Set Resolution */
	stts751_resolution_set(&dev_ctx, STTS751_12bit);

	return 0;
}
