#include "lv_port_indev.h"
#include "lvgl.h"
#include <esp_log.h>
#include "i2c.h"
#include "dev_board.h"

#define TAG "GT911"

#define CONFIG_GT911_I2C_TOUCH_PORT I2C_ZERO //使用到的I2C PORT
//触摸颠倒 ToDo->和屏幕定义一起自动处理
// #define CONFIG_LV_GT911_SWAPXY 1
#define CONFIG_LV_GT911_INVERT_X 1
#define CONFIG_LV_GT911_INVERT_Y 1

#define GT911_PRODUCT_ID_LEN   4

/* Register Map of GT911 */
#define GT911_PRODUCT_ID1             0x8140
#define GT911_PRODUCT_ID2             0x8141
#define GT911_PRODUCT_ID3             0x8142
#define GT911_PRODUCT_ID4             0x8143
#define GT911_FIRMWARE_VER_L          0x8144
#define GT911_FIRMWARE_VER_H          0x8145
#define GT911_X_COORD_RES_L           0x8146
#define GT911_X_COORD_RES_H           0x8147
#define GT911_Y_COORD_RES_L           0x8148
#define GT911_Y_COORD_RES_H           0x8149
#define GT911_VENDOR_ID               0x814A

#define GT911_STATUS_REG              0x814E
#define GT911_STATUS_REG_BUF          0x80
#define GT911_STATUS_REG_LARGE        0x40
#define GT911_STATUS_REG_PROX_VALID   0x20
#define GT911_STATUS_REG_HAVEKEY      0x10
#define GT911_STATUS_REG_PT_MASK      0x0F

#define GT911_TRACK_ID1               0x814F
#define GT911_PT1_X_COORD_L           0x8150
#define GT911_PT1_X_COORD_H           0x8151
#define GT911_PT1_Y_COORD_L           0x8152
#define GT911_PT1_Y_COORD_H           0x8153
#define GT911_PT1_X_SIZE_L            0x8154
#define GT911_PT1_X_SIZE_H            0x8155

typedef struct {
    bool inited;
    char product_id[GT911_PRODUCT_ID_LEN];
    uint16_t max_x_coord;
    uint16_t max_y_coord;
    uint8_t i2c_dev_addr;
} gt911_status_t;

gt911_status_t gt911_status;

//定义在外面防止内存被释放导致的无限重启错误
static lv_indev_drv_t indev_drv;

//TODO: handle multibyte read and refactor to just one read transaction
static esp_err_t gt911_i2c_read(uint8_t slave_addr, uint16_t register_addr, uint8_t *data_buf, uint8_t len) {
    return lvgl_i2c_read(CONFIG_GT911_I2C_TOUCH_PORT, slave_addr, register_addr | I2C_REG_16, data_buf, len);
}

static esp_err_t gt911_i2c_write8(uint8_t slave_addr, uint16_t register_addr, uint8_t data) {
    uint8_t buffer = data;
    return lvgl_i2c_write(CONFIG_GT911_I2C_TOUCH_PORT, slave_addr, register_addr | I2C_REG_16, &buffer, 1);
}

//在gt911初始化时通过设置相对于引脚的电平从而设定I2C的地址
static void gt911_set_addr(uint8_t dev_addr){
//fix show error the first time
		gpio_config_t io_conf;
		io_conf.intr_type = GPIO_INTR_DISABLE;
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = (1ULL << CONFIG_GT911_INT_PIN)|(1ULL << CONFIG_GT911_RST_PIN);
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		gpio_config(&io_conf);
		gpio_pad_select_gpio(CONFIG_GT911_INT_PIN);
		gpio_set_direction(CONFIG_GT911_INT_PIN, GPIO_MODE_OUTPUT);
		gpio_pad_select_gpio(CONFIG_GT911_RST_PIN);
		gpio_set_direction(CONFIG_GT911_RST_PIN, GPIO_MODE_OUTPUT);

		// 设置引脚电平
		gpio_set_level(CONFIG_GT911_INT_PIN, 0);
		gpio_set_level(CONFIG_GT911_RST_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(10));
		gpio_set_level(CONFIG_GT911_INT_PIN, GT911_I2C_SLAVE_ADDR==0x29);
		vTaskDelay(pdMS_TO_TICKS(1));
		gpio_set_level(CONFIG_GT911_RST_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(5));
		gpio_set_level(CONFIG_GT911_INT_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(50));
		vTaskDelay(pdMS_TO_TICKS(50));
}

/**
  * @brief  Initialize for GT911 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of GT911).
  * @retval None
  */
void gt911_init(uint8_t dev_addr) {
    if (!gt911_status.inited) {
        gt911_status.i2c_dev_addr = dev_addr;
        uint8_t data_buf;
        esp_err_t ret;
		gt911_set_addr(dev_addr);//fix show error the first time
        ESP_LOGI(TAG, "Checking for GT911 Touch Controller");
        if ((ret = gt911_i2c_read(dev_addr, GT911_PRODUCT_ID1, &data_buf, 1) != ESP_OK)) {
            ESP_LOGE(TAG, "Error reading from device: %s",
                        esp_err_to_name(ret));    // Only show error the first time
            return;
        }

        // Read 4 bytes for Product ID in ASCII
        for (int i = 0; i < GT911_PRODUCT_ID_LEN; i++) {
            gt911_i2c_read(dev_addr, (GT911_PRODUCT_ID1 + i), (uint8_t *)&(gt911_status.product_id[i]), 1);
        }
        ESP_LOGI(TAG, "\tProduct ID: %s", gt911_status.product_id);

        gt911_i2c_read(dev_addr, GT911_VENDOR_ID, &data_buf, 1);
        ESP_LOGI(TAG, "\tVendor ID: 0x%02x", data_buf);

        gt911_i2c_read(dev_addr, GT911_X_COORD_RES_L, &data_buf, 1);
        gt911_status.max_x_coord = data_buf;
        gt911_i2c_read(dev_addr, GT911_X_COORD_RES_H, &data_buf, 1);
        gt911_status.max_x_coord |= ((uint16_t)data_buf << 8);
        ESP_LOGI(TAG, "\tX Resolution: %d", gt911_status.max_x_coord);

        gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_L, &data_buf, 1);
        gt911_status.max_y_coord = data_buf;
        gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_H, &data_buf, 1);
        gt911_status.max_y_coord |= ((uint16_t)data_buf << 8);
        ESP_LOGI(TAG, "\tY Resolution: %d", gt911_status.max_y_coord);
        gt911_status.inited = true;
    }
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
static bool gt911_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint8_t touch_pnt_cnt;        // Number of detected touch points
    static int16_t last_x = 0;  // 12bit pixel value
    static int16_t last_y = 0;  // 12bit pixel value
    uint8_t data_buf;
    uint8_t status_reg;

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_STATUS_REG, &status_reg, 1);
//    ESP_LOGI(TAG, "\tstatus: 0x%02x", status_reg);
    touch_pnt_cnt = status_reg & 0x0F;
    if ((status_reg & 0x80) || (touch_pnt_cnt < 6)) {
        //Reset Status Reg Value
        gt911_i2c_write8(gt911_status.i2c_dev_addr, GT911_STATUS_REG, 0x00);
    }
    if (touch_pnt_cnt != 1) {    // ignore no touch & multi touch
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;
        return false;
    }

//    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_TRACK_ID1, &data_buf, 1);
//    ESP_LOGI(TAG, "\ttrack_id: %d", data_buf);

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_X_COORD_L, &data_buf, 1);
    last_x = data_buf;
    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_X_COORD_H, &data_buf, 1);
    last_x |= ((uint16_t)data_buf << 8);

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_Y_COORD_L, &data_buf, 1);
    last_y = data_buf;
    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_Y_COORD_H, &data_buf, 1);
    last_y |= ((uint16_t)data_buf << 8);

#if CONFIG_LV_GT911_INVERT_X
    last_x = gt911_status.max_x_coord - last_x;
#endif
#if CONFIG_LV_GT911_INVERT_Y
    last_y = gt911_status.max_y_coord - last_y;
#endif
#if CONFIG_LV_GT911_SWAPXY
    int16_t swap_buf = last_x;
    last_x = last_y;
    last_y = swap_buf;
#endif

    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_PR;
    ESP_LOGI(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    ESP_LOGV(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    return false;
}

//先调用gt911_init()初始化触摸再注册
void lv_port_indev_init(void){
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = &gt911_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
}


