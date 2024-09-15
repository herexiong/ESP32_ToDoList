#include <stdio.h>
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

//驱动
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "sd.h"
#include "audio_player.h"
//ui
#include "ui/ui_app/ui_app.h"
//dlna
#include "audio_dlna.h"
#include "audio_player.h"
//时间
#include "ntp_time.h"
//天气
#include "weather.h"
//连接
#include "protocol_examples_common.h"

#define TAG "main"
 
void print_task_info(void) {
    char buffer[1024];
    vTaskList(buffer);
    ESP_LOGI(TAG, "\nTask list:\nName          State   Prio    Stack   Num\n%s", buffer);
}

void print_task(void *param){
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        print_task_info();
    }
    
}

//初始化硬件
static void hardware_init(void){
    esp_log_level_set("GT911", ESP_LOG_NONE);//取消触摸日志打印
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    disp_8080_init();
    gt911_init(GT911_I2C_SLAVE_ADDR);
    sd_init();//初始化SD卡，LVGL对接文件系统在menuconfig内

    //网络初始化
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
}

void app_main(void)
{
    //初始化硬件
    hardware_init();
    //开启Dlna
    xTaskCreate(dlna_init_task,"dlna_start_task",4*1024,NULL,5,NULL);
    //播放功能
    xTaskCreate(esp_audio_task,"esp_audio_task",32*1024,NULL,5,NULL);
    //UI
    xTaskCreate(lv_task,"lvgl",16*1024,NULL,5,NULL);
    //时间任务
    xTaskCreate(time_task,"time_task",4*1024,NULL,5,NULL);
    //天气任务
    xTaskCreate(weather_task,"weather_task",8*1024,NULL,5,NULL);

    // vTaskDelay(pdMS_TO_TICKS(1000));
    // xTaskCreate(print_task,"print",8*1024,NULL,5,NULL);
}
