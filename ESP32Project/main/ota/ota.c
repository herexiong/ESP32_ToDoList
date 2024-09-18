#include "ota.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ghota.h>
#include "esp_spiffs.h"

#define TAG "OTA"

void mount_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("storage", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    // read_from_spiffs();
}

void unmount_spiffs() {
    esp_vfs_spiffs_unregister("storage");
}

static void ghota_event_callback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ghota_client_handle_t *client = (ghota_client_handle_t *)handler_args;
    ESP_LOGI(TAG, "Got Update Callback: %s", ghota_get_event_str(id));
    if (id == GHOTA_EVENT_START_STORAGE_UPDATE) {
        ESP_LOGI(TAG, "Starting storage update");
        /* if we are updating the SPIFF storage we should unmount it */
        unmount_spiffs();
    } else if (id == GHOTA_EVENT_FINISH_STORAGE_UPDATE) {
        ESP_LOGI(TAG, "Ending storage update");
        /* after updating we can remount, but typically the device will reboot shortly after recieving this event. */
        mount_spiffs();
    } else if (id == GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS) {
        /* display some progress with the firmware update */
        ESP_LOGI(TAG, "Firmware Update Progress: %d%%", *((int*) event_data));
    } else if (id == GHOTA_EVENT_STORAGE_UPDATE_PROGRESS) {
        /* display some progress with the spiffs partition update */
        ESP_LOGI(TAG, "Storage Update Progress: %d%%", *((int*) event_data));
    }
    (void)client;
    return;
}



void ota_task(void *param){
    /* initialize our ghota config */
    ghota_config_t ghconfig = {
        .filenamematch = OTAFILENAMEMATCH,
        //取消存储分区的OTA
        // .storagenamematch = "storage-esp32.bin",
        // .storagepartitionname = "storage",
        /* 1 minute as a example, but in production you should pick something larger (remember, Github has ratelimites on the API! )*/
        .updateInterval = 1,
    };
    /* initialize ghota. */
    ghota_client_handle_t *ghota_client = ghota_init(&ghconfig);
    if (ghota_client == NULL) {
        ESP_LOGE(TAG, "ghota_client_init failed");
        return;
    }
    /* register for events relating to the update progress */
    esp_event_handler_register(GHOTA_EVENTS, ESP_EVENT_ANY_ID, &ghota_event_callback, ghota_client);
#define DO_BACKGROUND_UPDATE 1
#define DO_FOREGROUND_UPDATE 0
#define DO_MANUAL_CHECK_UPDATE 0

#ifdef DO_BACKGROUND_UPDATE
    /* for private repositories or to get more API calls than anonymouse, set a github username and PAT token
     * see https://docs.github.com/en/github/authenticating-to-github/creating-a-personal-access-token
     * for more information on how to create a PAT token.
     * 
     * Be carefull, as the PAT token will be stored in your firmware etc and can be used to access your github account.
     */
    //ESP_ERROR_CHECK(ghota_set_auth(ghota_client, "<Insert GH Username>", "<insert PAT TOKEN>"));

    /* start a timer that will automatically check for updates based on the interval specified above */
    ESP_ERROR_CHECK(ghota_start_update_timer(ghota_client));

#elif DO_FORGROUND_UPDATE
    /* or do a check/update now
     * This runs in a new task under freeRTOS, so you can do other things while it is running.
     */
    ESP_ERROR_CHECK(ghota_start_update_task(ghota_client));

#elif DO_MANUAL_CHECK_UPDATE
    /* Alternatively you can do manual checks 
     * but note, you probably have to increase the Stack size for the task this runs on
     */

    /* Query the Github Release API for the latest release */
    ESP_ERROR_CHECK(ghota_check(ghota_client));

    /* get the semver version of the currently running firmware */
    semver_t *cur = ghota_get_current_version(ghota_client);
    if (cur) {
         ESP_LOGI(TAG, "Current version: %d.%d.%d", cur->major, cur->minor, cur->patch);
         semver_free(cur);
    

    /* get the version of the latest release on Github */
    semver_t *new = ghota_get_latest_version(ghota_client);
    if (new) {
        ESP_LOGI(TAG, "New version: %d.%d.%d", new->major, new->minor, new->patch);
        semver_free(new);
    }

    /* do some comparisions */
    if (semver_gt(new, cur) == 1) {
        ESP_LOGI(TAG, "New version is greater than current version");
    } else if (semver_eq(new, cur) == 1) {
        ESP_LOGI(TAG, "New version is equal to current version");
    } else {
        ESP_LOGI(TAG, "New version is less than current version");
    }

    /* assuming we have a new version, then do a actual update */
    ESP_ERROR_CHECK(ghota_update(ghota_client));
    /* if there was a new version installed, the esp will reboot after installation and will not reach this code */    
#endif

    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG, "This is where we do other things. Memory Dump Below to see the memory usage");
        // ESP_LOGI(TAG, "Memory: Free %dKiB Low: %dKiB\n", (int)xPortGetFreeHeapSize()/1024, (int)xPortGetMinimumEverFreeHeapSize()/1024);
    }
}

/*
在开启DLNA和天气时间时无法连接日志
I (68499) GHOTA: Firmware Update Task Starting
I (68499) GHOTA: Checking for new release
I (68499) GHOTA: Searching for Firmware from https://gitee.com/api/v5/repos/herexiong/ESP32_ToDoList/releases/latest
I (68499) OTA: Got Update Callback: GHOTA_EVENT_START_CHECK
E (68629) esp-tls-mbedtls: mbedtls_ssl_setup returned -0x7F00
E (68629) esp-tls: create_ssl_handle failed
E (68629) esp-tls: Failed to open new connection
E (68639) TRANSPORT_BASE: Failed to open a new connection
E (68649) HTTP_CLIENT: Connection failed, sock < 0
E (68649) GHOTA: HTTP GET request failed: ESP_ERR_HTTP_CONNECT
E (68659) GHOTA: Last esp error code: 0x8017
E (68659) GHOTA: Last mbedtls failure: 0x7f00
I (68669) GHOTA: No Update Available
I (68669) OTA: Got Update Callback: GHOTA_EVENT_NOUPDATE_AVAILABLE
I (68669) GHOTA: Firmware Update Task Finished
I (68679) OTA: Got Update Callback: GHOTA_EVENT_NOUPDATE_AVAILABLE
*/

/*
I (66341) GHOTA: Firmware Update Task Starting
I (66341) GHOTA: Checking for new release
I (66341) OTA: Got Update Callback: GHOTA_EVENT_START_CHECK
I (66341) GHOTA: Searching for Firmware from https://gitee.com/api/v5/repos/herexiong/ESP32_ToDoList/releases/latest
I (66581) esp-x509-crt-bundle: Certificate validated
I (67161) GHOTA: Current Version 0.0.0
I (67161) GHOTA: New Version 0.0.1
I (67161) GHOTA: Asset: ESP32_ToDoList.bin
I (67161) GHOTA: Firmware URL: https://gitee.com/herexiong/ESP32_ToDoList/releases/download/v0.0.1/ESP32_ToDoList.bin
I (67171) GHOTA: New Version Available
I (67171) OTA: Got Update Callback: GHOTA_EVENT_UPDATE_AVAILABLE
I (67171) GHOTA: Scheduled Check for Firmware Update Starting
I (67191) OTA: Got Update Callback: GHOTA_EVENT_START_UPDATE
I (68881) esp-x509-crt-bundle: Certificate validated
I (69521) HTTP_CLIENT: Body received in fetch header state, 0x3fcf3197, 157
I (69661) HTTP_CLIENT: Body received in fetch header state, 0x3fcf31d7, 229
I (69901) esp-x509-crt-bundle: Certificate validated
I (70651) HTTP_CLIENT: Body received in fetch header state, 0x3fcf32b9, 67
I (70651) esp_https_ota: Starting OTA...
I (70651) esp_https_ota: Writing to partition subtype 17 at offset 0x210000
I (70661) GHOTA: New Firmware Details:
I (70671) GHOTA: Project name: ESP32_ToDoList
I (70671) GHOTA: Firmware version: 35a8ff4-dirty
I (70681) GHOTA: Compiled time: Sep 17 2024 21:02:48
I (70681) GHOTA: ESP-IDF: v4.4.6-363-gc1c69210de-dirty
I (70691) GHOTA: SHA256:
I (70691) GHOTA: 79 df 0c d0 56 56 d0 54 9d ad 93 8a ef ab c5 d2
I (70701) GHOTA: 3d f1 2f 75 3e 24 97 9c 60 6b b0 c6 95 91 17 3b 
I (70761) OTA: Got Update Callback: GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS
I (70761) OTA: Firmware Update Progress: 0%

I (169647) esp_image: segment 0: paddr=00210020 vaddr=3c130020 size=9e260h (647776) map
I (169697) esp_image: segment 1: paddr=002ae288 vaddr=3fc9bce0 size=01d90h (  7568) 
I (169707) esp_image: segment 2: paddr=002b0020 vaddr=42000020 size=126c78h (1207416) map
I (169807) esp_image: segment 3: paddr=003d6ca0 vaddr=3fc9da70 size=02660h (  9824) 
I (169817) esp_image: segment 4: paddr=003d9308 vaddr=40374000 size=17cd4h ( 97492)
I (169827) OTA: Got Update Callback: GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS
I (169827) OTA: Firmware Update Progress: 100%
I (169827) esp_image: segment 0: paddr=00210020 vaddr=3c130020 size=9e260h (647776) map
I (169897) esp_image: segment 1: paddr=002ae288 vaddr=3fc9bce0 size=01d90h (  7568) 
I (169897) esp_image: segment 2: paddr=002b0020 vaddr=42000020 size=126c78h (1207416) map
I (170007) esp_image: segment 3: paddr=003d6ca0 vaddr=3fc9da70 size=02660h (  9824) 
I (170007) esp_image: segment 4: paddr=003d9308 vaddr=40374000 size=17cd4h ( 97492)
E (170017) esp_ota_ops: not found otadata
E (170017) esp_https_ota: esp_ota_set_boot_partition failed! err=0x105
E (170027) GHOTA: ESP_HTTPS_OTA upgrade failed 0x105
I (170027) OTA: Got Update Callback: GHOTA_EVENT_UPDATE_FAILED
I (170037) GHOTA: Firmware Update Task Finished
//第二次log
E (110799) esp_https_ota: esp_ota_set_boot_partition failed! err=0x102
E (110799) GHOTA: ESP_HTTPS_OTA upgrade failed 0x102
I (110799) OTA: Got Update Callback: GHOTA_EVENT_UPDATE_FAILED
I (110809) GHOTA: Firmware Update Task Finished

*/