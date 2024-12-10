#include "uart_app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "cJSON.h"

#include <stdio.h>
#include <string.h>

#include "ui.h"

static const char *TAG = "UART TEST";

#define UART_BUFFER_SIZE (1024)
#define UART_BAUD_RATE 115200
#define UART_PORT_NUM 2
#define UART_RXD_PIN 44
#define UART_TXD_PIN 43
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

void parse_uart_json(char *buffer);

void uart_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_RTS, UART_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(UART_BUFFER_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUFFER_SIZE - 1), 20 / portTICK_RATE_MS);
		
        // Write data back to the UART
        // uart_write_bytes(UART_PORT_NUM, (const char *) data, len);
        // if (len) {
        //     data[len] = '\0';
        //     ESP_LOGI(TAG, "Recv str: %s", (char *) data);
        // }
        if (len) 
            parse_uart_json((char *)data);
        vTaskDelay(pdMS_TO_TICKS(10));
		//ToDo->智能跳转
		//ToDo->根据厂商名字自动变色
    }
	free(data);
	vTaskDelete(NULL);
}

void parse_uart_json(char *buffer){
	if (buffer == NULL)
	{
		return;
	}
	cJSON *json_root = cJSON_Parse(buffer);
	if (json_root != NULL)
	{
		// CPU 数据解析
		cJSON *json_cpu = cJSON_GetObjectItem(json_root, "CPU");
		if (json_cpu) {
			cJSON *temp = cJSON_GetObjectItem(json_cpu, "title");
			char *cpuTitleStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_cpu, "temp");
			char *cpuTempStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_cpu, "power");
			char *cpuPowerStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_cpu, "usage");
			char *cpuUsageStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			if (cpuTitleStr && cpuTempStr && cpuPowerStr && cpuUsageStr) {
				MonitorCPU_ui_set(cpuTitleStr, cpuPowerStr, cpuUsageStr, cpuTempStr);
			}

			free(cpuTitleStr);
			free(cpuTempStr);
			free(cpuPowerStr);
			free(cpuUsageStr);
		}

		// GPU 数据解析
		cJSON *json_gpu = cJSON_GetObjectItem(json_root, "GPU");
		if (json_gpu) {
			cJSON *temp = cJSON_GetObjectItem(json_gpu, "title");
			char *gpuTitleStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_gpu, "temp");
			char *gpuTempStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_gpu, "power");
			char *gpuPowerStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_gpu, "usage");
			char *gpuUsageStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_gpu, "totalRAM");
			char *gpuTotalRamStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_gpu, "usedRAM");
			char *gpuUsedRamStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			if (gpuTitleStr && gpuTempStr && gpuPowerStr && gpuUsageStr && gpuTotalRamStr && gpuUsedRamStr) {
				MonitorGPU_ui_set(gpuTitleStr, gpuPowerStr, gpuUsageStr, gpuTempStr, gpuUsedRamStr, gpuTotalRamStr);
			}

			free(gpuTitleStr);
			free(gpuTempStr);
			free(gpuPowerStr);
			free(gpuUsageStr);
			free(gpuTotalRamStr);
			free(gpuUsedRamStr);
		}
		// RAM 数据解析
		cJSON *json_ram = cJSON_GetObjectItem(json_root, "RAM");
		if (json_ram) {
			cJSON *temp = cJSON_GetObjectItem(json_ram, "totalRAM");
			char *ramTotalRamStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_ram, "usage");
			char *ramUsageStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_ram, "usedRAM");
			char *ramUsedRamStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			if (ramTotalRamStr && ramUsageStr && ramUsedRamStr) {
				MonitorRAM_ui_set(ramUsageStr, ramUsedRamStr, ramTotalRamStr);
			}

			free(ramUsageStr);
			free(ramUsedRamStr);
			free(ramTotalRamStr);
		}
		// NET 数据解析
		cJSON *json_net = cJSON_GetObjectItem(json_root, "NET");
		if (json_net) {
			cJSON *temp = cJSON_GetObjectItem(json_net, "title");
			char *netTitleStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_net, "upload");
			char *netupStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			temp = cJSON_GetObjectItem(json_net, "download");
			char *netdwStr = temp && temp->valuestring ? strdup(temp->valuestring) : NULL;

			if (netTitleStr && netupStr && netdwStr) {
				MonitorNET_ui_set(netTitleStr, netupStr, netdwStr);
			}

			free(netTitleStr);
			free(netupStr);
			free(netdwStr);
		}
		cJSON_Delete(json_root);

	}
	
}
