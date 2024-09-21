#ifndef LV_PORT_INDEV_H_
#define LV_PORT_INDEV_H_

#include <stdint.h>

#define GT911_I2C_SLAVE_ADDR   0x5D

void gt911_init(uint8_t dev_addr);
void lv_port_indev_init(void);

#endif