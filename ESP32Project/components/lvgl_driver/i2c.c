#include "i2c.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "dev_board.h"

#define TAG "I2C_Driver"

static const uint8_t ACK_CHECK_EN = 1;

static QueueHandle_t lvgl_i2c_local_mutex[2] = { NULL, NULL };
static QueueHandle_t* lvgl_i2c_mutex = &lvgl_i2c_local_mutex[0];

static void i2c_send_address(i2c_cmd_handle_t cmd, uint16_t addr, i2c_rw_t rw) {
	if (addr & I2C_ADDR_10) {
		i2c_master_write_byte(cmd, 0xF0 | ((addr & 0x3FF) >> 7) | rw, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, addr & 0xFF, ACK_CHECK_EN);
	} else {
		i2c_master_write_byte(cmd, (addr << 1) | rw, ACK_CHECK_EN);
	}
}

static void i2c_send_register(i2c_cmd_handle_t cmd, uint32_t reg) {
	if (reg & I2C_REG_16) {
	    i2c_master_write_byte(cmd, (reg & 0xFF00) >> 8, ACK_CHECK_EN);
	}
    i2c_master_write_byte(cmd, reg & 0xFF, ACK_CHECK_EN);
}

static esp_err_t lvgl_i2c_force_unlock(i2c_port_t port) {
	I2C_PORT_CHECK(port, ESP_FAIL);
	if (lvgl_i2c_mutex[port]) {
		vSemaphoreDelete(lvgl_i2c_mutex[port]);
	}
	lvgl_i2c_mutex[port] = xSemaphoreCreateMutex();
	return ESP_OK;
}

static esp_err_t lvgl_i2c_lock(i2c_port_t port) {
	I2C_PORT_CHECK(port, ESP_FAIL);
	ESP_LOGV(TAG, "Mutex lock set for %d.", (int)port);

	TickType_t timeout;
	#if defined (I2C_ZERO)
		if (port == I2C_NUM_0) {
			timeout = (CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif
	#if defined (I2C_ONE)
		if (port == I2C_NUM_1) {
			timeout = (CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif

	if (xSemaphoreTake(lvgl_i2c_mutex[port], timeout) == pdTRUE) {
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "Removing stale mutex lock from port %d.", (int)port);
		lvgl_i2c_force_unlock(port);
		return (xSemaphoreTake(lvgl_i2c_mutex[port], timeout) == pdTRUE ? ESP_OK : ESP_FAIL);
	}
}

static esp_err_t lvgl_i2c_unlock(i2c_port_t port) {
	I2C_PORT_CHECK(port, ESP_FAIL);
	ESP_LOGV(TAG, "Mutex lock removed for %d.", (int)port);
	return (xSemaphoreGive(lvgl_i2c_mutex[port]) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t lvgl_i2c_init(int port) {

	I2C_PORT_CHECK(port, ESP_FAIL);

	esp_err_t ret = ESP_OK;

	if (lvgl_i2c_mutex[port] == 0) {

		ESP_LOGI(TAG, "Starting I2C master at port %d.", (int)port);

		lvgl_i2c_mutex[port] = xSemaphoreCreateMutex();

		i2c_config_t conf = {0};
		
		#ifdef HAS_CLK_FLAGS
			conf.clk_flags = 0;
		#endif

		#if defined (I2C_ZERO)
			if (port == I2C_NUM_0) {
				conf.sda_io_num = CONFIG_I2C_MANAGER_0_SDA;
				conf.scl_io_num = CONFIG_I2C_MANAGER_0_SCL;
				conf.sda_pullup_en = I2C_MANAGER_0_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
				conf.scl_pullup_en = conf.sda_pullup_en;
				conf.master.clk_speed = CONFIG_I2C_MANAGER_0_FREQ_HZ;
			}
		#endif

		#if defined (I2C_ONE)
			if (port == I2C_NUM_1) {
				conf.sda_io_num = CONFIG_I2C_MANAGER_1_SDA;
				conf.scl_io_num = CONFIG_I2C_MANAGER_1_SCL;
				conf.sda_pullup_en = I2C_MANAGER_1_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
				conf.scl_pullup_en = conf.sda_pullup_en;
				conf.master.clk_speed = CONFIG_I2C_MANAGER_1_FREQ_HZ;
			}
		#endif

		conf.mode = I2C_MODE_MASTER;

		ret = i2c_param_config(port, &conf);
		ret |= i2c_driver_install(port, conf.mode, 0, 0, 0);

		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to initialise I2C port %d.", (int)port);
			ESP_LOGW(TAG, "If it was already open, we'll use it with whatever settings were used "
			              "to open it. See I2C Manager README for details.");
		} else {
			ESP_LOGI(TAG, "Initialised port %d (SDA: %d, SCL: %d, speed: %d Hz.)",
					 port, conf.sda_io_num, conf.scl_io_num, conf.master.clk_speed);
		}

	}

    return ret;
}

/// @brief 
/// @param port I2C端口
/// @param addr 从机地址
/// @param reg 读取的寄存器地址
/// @param buffer 读取数据存储缓存地址
/// @param size 需要读取数据的大小
/// @return 
esp_err_t lvgl_i2c_read(int port, uint16_t addr, uint32_t reg, uint8_t *buffer, uint16_t size) {

	I2C_PORT_CHECK(port, ESP_FAIL);

    esp_err_t result;

    // May seem weird, but init starts with a check if it's needed, no need for that check twice.
	lvgl_i2c_init(port);

   	ESP_LOGV(TAG, "Reading port %d, addr 0x%03x, reg 0x%04x", port, addr, reg);

	TickType_t timeout = 0;
	#if defined (I2C_ZERO)
		if (port == I2C_NUM_0) {
			timeout = I2C_MANAGER_0_TIMEOUT;
		}
	#endif
	#if defined (I2C_ONE)
		if (port == I2C_NUM_1) {
			timeout = I2C_MANAGER_1_TIMEOUT;
		}
	#endif

	if (lvgl_i2c_lock((int)port) == ESP_OK) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		if (!(reg & I2C_NO_REG)) {
			/* When reading specific register set the addr pointer first. */
			i2c_master_start(cmd);
			i2c_send_address(cmd, addr, I2C_MASTER_WRITE);
			i2c_send_register(cmd, reg);
		}
		/* Read size bytes from the current pointer. */
		i2c_master_start(cmd);
		i2c_send_address(cmd, addr, I2C_MASTER_READ);
		i2c_master_read(cmd, buffer, size, I2C_MASTER_LAST_NACK);
		i2c_master_stop(cmd);
		result = i2c_master_cmd_begin(port, cmd, timeout);
		i2c_cmd_link_delete(cmd);
		lvgl_i2c_unlock((int)port);
	} else {
		ESP_LOGE(TAG, "Lock could not be obtained for port %d.", (int)port);
		return ESP_ERR_TIMEOUT;
	}

    if (result != ESP_OK) {
    	ESP_LOGW(TAG, "Error: %d", result);
    }

	ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_VERBOSE);

    return result;
}

/// @brief 
/// @param port I2C端口
/// @param addr 从设备地址
/// @param reg 写的寄存器地址？
/// @param buffer 写入的数据
/// @param size 写入的大小
/// @return 
esp_err_t lvgl_i2c_write(i2c_port_t port, uint16_t addr, uint32_t reg, const uint8_t *buffer, uint16_t size) {

	I2C_PORT_CHECK(port, ESP_FAIL);

    esp_err_t result;

    // May seem weird, but init starts with a check if it's needed, no need for that check twice.
	lvgl_i2c_init(port);

    ESP_LOGV(TAG, "Writing port %d, addr 0x%03x, reg 0x%04x", port, addr, reg);

	TickType_t timeout = 0;
	#if defined (I2C_ZERO)
		if (port == I2C_NUM_0) {
			timeout = (CONFIG_I2C_MANAGER_0_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif
	#if defined (I2C_ONE)
		if (port == I2C_NUM_1) {
			timeout = (CONFIG_I2C_MANAGER_1_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif

	if (lvgl_i2c_lock((int)port) == ESP_OK) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_send_address(cmd, addr, I2C_MASTER_WRITE);
		if (!(reg & I2C_NO_REG)) {
			i2c_send_register(cmd, reg);
		}
		i2c_master_write(cmd, (uint8_t *)buffer, size, ACK_CHECK_EN);
		i2c_master_stop(cmd);
		result = i2c_master_cmd_begin( port, cmd, timeout);
		i2c_cmd_link_delete(cmd);
		lvgl_i2c_unlock((int)port);
	} else {
		ESP_LOGE(TAG, "Lock could not be obtained for port %d.", (int)port);
		return ESP_ERR_TIMEOUT;
	}

    if (result != ESP_OK) {
    	ESP_LOGW(TAG, "Error: %d", result);
    }

	ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_VERBOSE);

    return result;
}