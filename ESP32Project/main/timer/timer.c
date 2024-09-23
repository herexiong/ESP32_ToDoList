#include "timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "../ui/ui.h"

#define TAG "USERTIMER"

static TimerHandle_t timer_handle = NULL;
static int secNum = 0;
static TaskHandle_t timer_task_handle;

static void timeCallBackTask(void)
{
	secNum++;
}

//定时器监测任务
static void timer_task(void *param){
	int mode     = *(((int *)param)+0);
	int secValue = *(((int *)param)+1);
	ESP_LOGI(TAG,"timer_task mode:%d secValue:%d",mode,secValue);
	if(mode == 0){//倒计时模式
		while (secNum <= secValue)
		{
			//倒计时未结束，更新UI
			int time = secValue - secNum;
			int hour = time/3600;
			int min  = (time - hour*3600)/60;
			int sec  = (time - hour*3600 -min*60);
			Countdown_ui_set(hour , min , sec , 0);
			// ESP_LOGI(TAG,"Countdown_ui_set time:%d hour:%d min:%d sec:%d",time,hour,min,sec);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		//倒计时结束
		Countdown_ui_set( 0 , 10 , 0 , 0);
		Countdown_ui_set( 0 , 0 , 0 , 1);
		ESP_LOGI(TAG,"timer_task end");
	}else if(*(int *)param == 1){//计时模式

	}
	xTimerStop(timer_handle,0); //停止定时器
	timer_task_handle = NULL;
	vTaskDelete(NULL);
}

//创建定时器
static void timer_init(void){
	timer_handle = xTimerCreate("timer",pdMS_TO_TICKS(1000),pdTRUE,(void*)1,(TimerCallbackFunction_t)timeCallBackTask );
	ESP_LOGI(TAG,"timer init");
}

/// @brief 启动定时器
/// @param mode 0 -> 倒计时模式，1 -> 计时模式
/// @param secValue 需要倒计时的秒数
void timer_start(int mode , int secValue){
	int param[2] = {mode,secValue};
	secNum = 0;
	if (timer_handle == NULL)
	{
		timer_init();
	}
	if (timer_handle != NULL)
	{
		xTimerStart(timer_handle,0); //开启定时器
		xTaskCreate(timer_task,"ota_task",4*1024,(void*)param,5,&timer_task_handle);
		ESP_LOGI(TAG,"timer start");
	}
}

/// @brief 暂停或关闭定时器
/// @param mode 0->暂停定时器 1->恢复定时器 2->杀掉定时器
/// @param secValue 
void timer_change_state(int mode){
	switch (mode)
	{
	case 0: //暂停定时器
		xTimerStop(timer_handle,0); //关闭定时器
		break;
	case 1: //恢复定时器
		if (timer_handle != NULL)
		{
			xTimerStart(timer_handle,0); //开启定时器
			ESP_LOGI(TAG,"timer start");
		}
		break;
	case 2: //杀掉定时器
		if (timer_task_handle != NULL)
		{
			vTaskDelete(timer_task_handle);
			timer_task_handle = NULL;
		}
		break;
	default:
		break;
	}
}