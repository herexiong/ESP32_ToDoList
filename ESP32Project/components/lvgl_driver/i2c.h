#ifndef I2C_H_
#define I2C_H_

#include <driver/i2c.h>
#include "esp_err.h"

////////////////////////////////////////////
#define I2C_ZERO I2C_NUM_0
#define CONFIG_I2C_MANAGER_0_ENABLED 1

// #define I2C_ONE I2C_NUM_1
// #define CONFIG_I2C_MANAGER_1_ENABLED 0

#define CONFIG_I2C_MANAGER_0_FREQ_HZ 400000
#define CONFIG_I2C_MANAGER_0_TIMEOUT 20
#define CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT 50
#define CONFIG_I2C_MANAGER_0_PULLUPS y

#define CONFIG_I2C_MANAGER_1_FREQ_HZ 400000
#define CONFIG_I2C_MANAGER_1_TIMEOUT 20
#define CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT 50
#define CONFIG_I2C_MANAGER_1_PULLUPS y

#if defined (I2C_NUM_0) && defined (CONFIG_I2C_MANAGER_0_ENABLED)
	#define I2C_ZERO 					I2C_NUM_0
	#if defined (CONFIG_I2C_MANAGER_0_PULLUPS)
		#define I2C_MANAGER_0_PULLUPS 	true
	#else
		#define I2C_MANAGER_0_PULLUPS 	false
	#endif

	#define I2C_MANAGER_0_TIMEOUT 		( CONFIG_I2C_MANAGER_0_TIMEOUT / portTICK_RATE_MS )
	#define I2C_MANAGER_0_LOCK_TIMEOUT	( CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT / portTICK_RATE_MS )
#endif

#if defined (I2C_NUM_1) && defined (CONFIG_I2C_MANAGER_1_ENABLED)
	#define I2C_ONE 					I2C_NUM_1
	#if defined (CONFIG_I2C_MANAGER_1_PULLUPS)
		#define I2C_MANAGER_1_PULLUPS 	true
	#else
		#define I2C_MANAGER_1_PULLUPS 	false
	#endif

	#define I2C_MANAGER_1_TIMEOUT 		( CONFIG_I2C_MANAGER_1_TIMEOUT / portTICK_RATE_MS )
	#define I2C_MANAGER_1_LOCK_TIMEOUT	( CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT / portTICK_RATE_MS )
#endif

#define ERROR_PORT(port, fail) { \
	ESP_LOGE(TAG, "Invalid port or not configured for I2C Manager: %d", (int)port); \
	return fail; \
}

#if defined(I2C_ZERO) && defined (I2C_ONE)
	#define I2C_PORT_CHECK(port, fail) \
		if (port != I2C_NUM_0 && port != I2C_NUM_1) ERROR_PORT(port, fail);
#else
	#if defined(I2C_ZERO)
		#define I2C_PORT_CHECK(port, fail) \
			if (port != I2C_NUM_0) ERROR_PORT(port, fail);
	#elif defined(I2C_ONE)
		#define I2C_PORT_CHECK(port, fail) \
			if (port != I2C_NUM_1) ERROR_PORT(port, fail);
	#else
		#define I2C_PORT_CHECK(port, fail) \
			ERROR_PORT(port, fail);
	#endif
#endif

#define I2C_ADDR_10 ( 1 << 15 )
#define I2C_REG_16  ( 1 << 31 )
#define I2C_NO_REG  ( 1 << 30 )
////////////////////////////////////////////
esp_err_t lvgl_i2c_init(int port);

esp_err_t lvgl_i2c_read(int port, uint16_t addr, uint32_t reg, uint8_t *buffer, uint16_t size);
esp_err_t lvgl_i2c_write(i2c_port_t port, uint16_t addr, uint32_t reg, const uint8_t *buffer, uint16_t size);

#endif