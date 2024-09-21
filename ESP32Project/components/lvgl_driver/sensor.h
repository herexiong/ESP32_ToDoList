#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

//SHT30
#define SHT30_I2C_SLAVE_ADDR      0x44

#define CMD_FETCH_DATA_H    0x22   //循环采样，参考sht30 datasheet
#define CMD_FETCH_DATA_L    0x36

//SGP30
#define SGP30_I2C_SLAVE_ADDR  (0x58)
#define SGP30_FEATURESET 0x0020    /**< The required set for this library */
#define SGP30_CRC8_POLYNOMIAL 0x31 /**< Seed for SGP30's CRC polynomial */
#define SGP30_CRC8_INIT 0xFF       /**< Init value for CRC */
#define SGP30_WORD_LEN 2           /**< 2 bytes per word */

void Sensor_task(void *param);

#endif