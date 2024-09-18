#include "weather.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "ui.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "ui.h"

#define TAG "weather"
#define CITY "chengdu"
#define PRIKEY "Sw1XQBXZhdSqUSpCw"
#define MAX_HTTP_OUTPUT_BUFFER 2048

void parse_weather_json(char *buffer){
	if (buffer == NULL)
	{
		return;
	}
	cJSON *json_root = cJSON_Parse(buffer);
	if (json_root != NULL)
	{
		cJSON *json_arr = cJSON_GetObjectItem(json_root,"results");

		cJSON *json_info = cJSON_GetArrayItem(json_arr,0);
		cJSON *json_now = cJSON_GetObjectItem(json_info,"now");

		cJSON *temp = cJSON_GetObjectItem(json_now,"text");
		char *weatherStr = (char *)malloc(sizeof(char)*strlen(temp->valuestring)+1);
		strcpy(weatherStr,temp->valuestring);
		// ESP_LOGE(TAG, "%s",weatherStr);

		temp = cJSON_GetObjectItem(json_now,"code");
		char *codeStr = (char *)malloc(sizeof(char)*strlen(temp->valuestring)+1);
		strcpy(codeStr,temp->valuestring);
		// ESP_LOGE(TAG, "%s",codeStr);

		temp = cJSON_GetObjectItem(json_now,"temperature");
		char *tempStr = (char *)malloc(sizeof(char)*strlen(temp->valuestring)+1);
		strcpy(tempStr,temp->valuestring);
		// ESP_LOGE(TAG, "%s",tempStr);

        cJSON *json_location = cJSON_GetObjectItem(json_info,"location");
		temp = cJSON_GetObjectItem(json_location,"name");
		char *cityStr = (char *)malloc(sizeof(char)*strlen(temp->valuestring)+1);
		strcpy(cityStr,temp->valuestring);
		// ESP_LOGE(TAG, "%s",cityStr);

		weather_ui_set(weatherStr,codeStr,tempStr,cityStr);

		cJSON_Delete(json_root);
		free(weatherStr);
		free(codeStr);
		free(tempStr);
        free(cityStr);
	}
	
}

/*
 *  http_native_request() demonstrates use of low level APIs to connect to a server,
 *  make a http request and read response. Event handler is not used in this case.
 *  Note: This approach should only be used in case use of low level APIs is required.
 *  The easiest way is to use esp_http_perform()s
 */
static void http_native_request(void)
{
    char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   // Buffer to store response of http request
    int content_length = 0;
    esp_http_client_config_t config = {
        .url = "http://api.seniverse.com/v3/weather/now.json?key="PRIKEY"&location="CITY"&language=zh-Hans&unit=c",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET Request
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
				// ESP_LOGI(TAG, "%s",output_buffer);
				parse_weather_json(output_buffer);
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, data_read);
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_close(client);
}

void weather_task(void *param){
    //获取天气
    while (1)
    {
		http_native_request();
        vTaskDelay(pdMS_TO_TICKS(300000));//300秒更新一次
    }
    vTaskDelete(NULL);
}