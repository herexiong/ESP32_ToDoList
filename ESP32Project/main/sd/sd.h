#ifndef SD_H_
#define SD_H_

//SD驱动
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"

// #define LV_FS_PATH "/storage" //文件系统挂载路径
#define LV_FS_PATH "/sdcard"

#define SDMMC 1 			  //使用SDIO
#define SDSPI 0

extern sdmmc_card_t *card;

esp_err_t sd_init(void);

typedef struct
{
	char *ssid;
	char *pwd;
	char *todoist_auth;
	char *todoist_prjid;
} todolist_syscfg_t;

void sd_read_param(todolist_syscfg_t* cfg);


#endif