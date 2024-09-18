#include "ui_app.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lvgl.h"
#include "ui.h"

#define TAG "ui_app"
//时间
#include "ntp_time.h"
//天气
#include "weather.h"

static void lv_tick_task(void *arg)
{
    lv_tick_inc(1);
}

static void lvgl_driver_init()
{
    lv_port_disp_init();
    lv_port_indev_init();
} 

void lv_task(void *param)
{
    lv_init();
    /* Initialize SCREEN TOUCH and SD */
    lvgl_driver_init();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1 * 1000));

    ui_init();
    lv_port_disp_backlight(true);//在UI初始化后打开背光避免花屏
    //时间任务
    xTaskCreate(time_task,"time_task",4*1024,NULL,5,NULL);
    //天气任务
    xTaskCreate(weather_task,"weather_task",8*1024,NULL,5,NULL);
    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
        
    }
    vTaskDelete(NULL);
}