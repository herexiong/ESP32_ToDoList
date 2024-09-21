#include "sensor.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "ui.h"

#define TAG "SENSOR"

typedef struct sgp30_dev {
    /**< The 48-bit serial number, this value is set when you call sgp30_init */
    uint16_t serial_number[3];
    /**< The last measurement of the IAQ-calculated Total Volatile Organic
            Compounds in ppb. This value is set when you call IAQmeasure() **/
    uint16_t TVOC;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call IAQmeasure() */
    uint16_t eCO2;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call IAQmeasureRaw() */
    uint16_t raw_H2;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This 
            value is set when you call IAQmeasureRaw */
    uint16_t raw_ethanol;
    /**< Interface pointer, used to store I2C address of the device */
    uint8_t i2c_addr;
} sgp30_dev_t;
//创建一个SGP30对象
sgp30_dev_t main_sgp30_sensor;

static uint8_t sht30_buf[6]={0};       //用于接收SHT30返回数据

//===============================================================================================================SHT30部分
static int sht30_init(void)
{
    const uint8_t data_buffer[2] = {CMD_FETCH_DATA_H,CMD_FETCH_DATA_L};
    return lvgl_i2c_write(I2C_ZERO,SHT30_I2C_SLAVE_ADDR,I2C_NO_REG,data_buffer,2);
}

static unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes)
{
	unsigned char bit;        // bit mask
    unsigned char crc = 0xFF; // calculated checksum
    unsigned char byteCtr;    // byte counter
    unsigned int POLYNOMIAL =  0x131;           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}

static unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
    unsigned char crc;
	crc = SHT3X_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}

static int sht30_get_value(void)
{
    int ret;
    ret = lvgl_i2c_read(I2C_ZERO,SHT30_I2C_SLAVE_ADDR,I2C_NO_REG,sht30_buf,6);
    if(ret!=ESP_OK)     //判断是否发送成功
    {
        return ret;
    }
    //校验读出来的数据，算法参考sht30 datasheet
    if( (!SHT3X_CheckCrc(sht30_buf,2,sht30_buf[2])) && (!SHT3X_CheckCrc(sht30_buf+3,2,sht30_buf[5])) )
    {
        ret = ESP_OK;//成功
    }
    else
    {
        ret = 1;
    }
    return ret;
}

//===============================================================================================================SGP30部分

/***********************
 * I²C 16-bit Commands *
 ***********************/
static uint8_t INIT_AIR_QUALITY[2] =        { 0x20, 0x03 };
static uint8_t MEASURE_AIR_QUALITY[2] =     { 0x20, 0x08 };
static uint8_t GET_BASELINE[2] =            { 0x20, 0x15 };
static uint8_t SET_BASELINE[2] =            { 0x20, 0x1E };
static uint8_t SET_HUMIDITY[2] =            { 0x20, 0x61 };
static uint8_t MEASURE_TEST[2] =            { 0x20, 0x32 };
static uint8_t GET_FEATURE_SET_VERSION[2] = { 0x20, 0x2F };
static uint8_t MEASURE_RAW_SIGNALS[2] =     { 0x20, 0x50 };
static uint8_t GET_SERIAL_ID[2] =           { 0x36, 0x82 };
static uint8_t SOFT_RESET[2] =              { 0x00, 0x06 };

/**
 * @brief Calculates 8-Bit checksum with given polynomial, used to validate SGP30 commands. 
 * 
 * @returns 8-bit checksum
 */
static uint8_t sgp30_calculate_CRC(uint8_t *data, uint8_t len) {
    /**
     ** Data and commands are protected with a CRC checksum to increase the communication reliability.
     ** The 16-bit commands that are sent to the sensor already include a 3-bit CRC checksum.
     ** Data sent from and received by the sensor is always succeeded by an 8-bit CRC.
     *! In write direction it is mandatory to transmit the checksum, since the SGP30 only accepts data if
     *! it is followed by the correct checksum. 
     *
     ** In read direction it is up to the master to decide if it wants to read and process the checksum
    */
    uint8_t crc = SGP30_CRC8_INIT;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ SGP30_CRC8_POLYNOMIAL;
        else
            crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief Executes commands based on SGP30 Command Table 
 * 
 * @param device        Pointer to sgp30 device
 * @param command       Command to be executed'
 * @param command_len   Command lenght
 * @param delay         Time to wait for a response
 * @param read_data     Buffer where read data will be stored
 * @param read_len      Size of read_data buffer
 * 
 * @see https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Datasheets/Sensirion_Gas_Sensors_SGP30_Datasheet.pdf
 *       Table #10
 */
static esp_err_t sgp30_execute_command(sgp30_dev_t *device, uint8_t command[], uint8_t command_len, uint16_t delay, uint16_t *read_data, uint8_t read_len) {

    /*********************************************************************************************
     ** Measurement routine: START condition, the I2C WRITE header (7-bit I2C device address plus 0
     ** as the write bit) and a 16-bit measurement command.
     **
     ** All commands are listed in TABLE 10 on the datasheet. 
     ** 
     ** 
     ** After the sensor has completed the measurement, the master can read the measurement results by 
     ** sending a START condition followed by an I2C READ header. The sensor will respond with the data.
     * 
     *! Each byte must be acknowledged by the microcontroller with an ACK condition for the sensor to continue sending data.
     *! If the sensor does not receive an ACK from the master after any byte of data, it will not continue sending data.
    **********************************************************************************************/

    esp_err_t err;

    // Writes SGP30 Command
    err = lvgl_i2c_write(I2C_ZERO,device->i2c_addr,I2C_NO_REG,command,command_len);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SGP30 I2C command! err: 0x%02x", err);
        return err;  
    }

    // Waits for device to process command and measure desired value
    vTaskDelay(delay / portTICK_RATE_MS);

    // Checks if there is data to be read from the user, (or if it's just a simple command write)
    if (read_len == 0) {
        return ESP_OK;
    }

    uint8_t reply_len = read_len * (SGP30_WORD_LEN + 1);
    uint8_t reply_buffer[reply_len];

    // Tries to read device reply
    err = lvgl_i2c_read(I2C_ZERO,device->i2c_addr,I2C_NO_REG,reply_buffer,reply_len);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SGP30 I2C command reply! err: 0x%02x", err);
        return err;  // failed to read reply buffer from chip
    }

    // Calculates expected CRC and compares it with the response
    for (uint8_t i = 0; i < read_len; i++) {
        uint8_t crc = sgp30_calculate_CRC(reply_buffer + i * 3, 2);
        ESP_LOGD(TAG, "%s - Calc CRC: %02x,   Reply CRC: %02x", __FUNCTION__, crc, reply_buffer[i * 3 + 2]);

        if (crc != reply_buffer[i * 3 + 2]) {
            ESP_LOGW(TAG, "Reply and Calculated CRCs are different");
            return false;
        }

        // If CRCs are equal, save data
        read_data[i] = reply_buffer[i * 3];
        read_data[i] <<= 8;
        read_data[i] |= reply_buffer[i * 3 + 1];
        ESP_LOGD(TAG, "%s - Read data: %04x", __FUNCTION__, read_data[i]);
    }

    return ESP_OK;
}

static void sgp30_IAQ_init(sgp30_dev_t *sensor) {
    sgp30_execute_command(sensor, INIT_AIR_QUALITY, 2, 10, NULL, 0);
}

static void sgp30_init(sgp30_dev_t *sensor) {
    sensor->i2c_addr = SGP30_I2C_SLAVE_ADDR;

    sgp30_execute_command(sensor, GET_SERIAL_ID, 2, 10, sensor->serial_number, 3);

    ESP_LOGI(TAG, "%s - Serial Number: %02x %02x %02x", __FUNCTION__, sensor->serial_number[0],
                                sensor->serial_number[1], sensor->serial_number[2]);

    uint16_t featureset;
    sgp30_execute_command(sensor, GET_FEATURE_SET_VERSION, 2, 10, &featureset, 1);
    ESP_LOGI(TAG, "%s - Feature set version: %04x", __FUNCTION__, featureset);

    sgp30_IAQ_init(sensor);
}

static void sgp30_softreset(sgp30_dev_t *sensor) {
    sgp30_execute_command(sensor, SOFT_RESET, 2, 10, NULL, 0);
}

static void sgp30_IAQ_measure(sgp30_dev_t *sensor) {
    uint16_t reply[2];

    sgp30_execute_command(sensor, MEASURE_AIR_QUALITY, 2, 20, reply, 2);
    sensor->TVOC = reply[1];
    sensor->eCO2 = reply[0];
}

static void sgp30_IAQ_measure_raw(sgp30_dev_t *sensor) {
    uint16_t reply[2];

    sgp30_execute_command(sensor, MEASURE_RAW_SIGNALS, 2, 20, reply, 2);
    sensor->raw_ethanol = reply[1];
    sensor->raw_H2 = reply[0];
}

static void sgp30_get_IAQ_baseline(sgp30_dev_t *sensor, uint16_t *eco2_base, uint16_t *tvoc_base) {
    uint16_t reply[2];

    sgp30_execute_command(sensor, GET_BASELINE, 2, 20, reply, 2);

    *eco2_base = reply[0];
    *tvoc_base = reply[1];
}

static void sgp30_set_IAQ_baseline(sgp30_dev_t *sensor, uint16_t eco2_base, uint16_t tvoc_base) {
    uint8_t baseline_command[8];

    baseline_command[0] = SET_BASELINE[0];
    baseline_command[1] = SET_BASELINE[1];

    baseline_command[2] = tvoc_base >> 8;
    baseline_command[3] = tvoc_base & 0xFF;
    baseline_command[4] = sgp30_calculate_CRC(baseline_command + 2, 2);

    baseline_command[5] = eco2_base >> 8;
    baseline_command[6] = eco2_base & 0xFF;
    baseline_command[7] = sgp30_calculate_CRC(baseline_command + 5, 2);

    sgp30_execute_command(sensor, baseline_command, 8, 20, NULL, 0);
}

static void sgp30_set_humidity(sgp30_dev_t *sensor, uint32_t absolute_humidity) {
    if (absolute_humidity > 256000) {
        ESP_LOGW(TAG, "%s - Abs humidity value %d is too high!", __FUNCTION__, absolute_humidity);
        return;
    }

    uint8_t ah_command[5];
    uint16_t ah_scaled = (uint16_t)(((uint64_t)absolute_humidity * 256 * 16777) >> 24);

    ah_command[0] = SET_HUMIDITY[0];
    ah_command[1] = SET_HUMIDITY[1];

    ah_command[2] = ah_scaled >> 8;
    ah_command[3] = ah_scaled & 0xFF;
    ah_command[4] = sgp30_calculate_CRC(ah_command + 2, 2);

    sgp30_execute_command(sensor, ah_command, 5, 20, NULL, 0);
}

void Sensor_task(void *param){
    while(sht30_init()!=ESP_OK)
        vTaskDelay(pdMS_TO_TICKS(10));
    //SGP30初始化
    sgp30_init(&main_sgp30_sensor);

    float tempData, humData;
    //根据SGP30 datasheet说明SGP30需要每1s读一次，初始化时发送TVOC = 400 14次
    for (int i = 0; i < 14; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        sgp30_IAQ_measure(&main_sgp30_sensor);
        // ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);

        if(sht30_get_value()==ESP_OK)   //获取温湿度
        {
            //算法参考sht30 datasheet
            tempData =( ( (  (sht30_buf[0]*256) +sht30_buf[1]) *175   )/65535.0  -45  );
            humData =  ( ( (sht30_buf[3]*256) + (sht30_buf[4]) )*100/65535.0) ;
            // ESP_LOGI("SHT30", "temp:%4.2f C   hum:%4.2f %%RH \r\n", tempData, humData); //℃打印出来是乱码
        }
    }

    //读取初始基线
    uint16_t eco2_baseline, tvoc_baseline;
    sgp30_get_IAQ_baseline(&main_sgp30_sensor, &eco2_baseline, &tvoc_baseline);
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);
    
    while(1){
        
        if(sht30_get_value()==ESP_OK)   //获取温湿度
        {
            //算法参考sht30 datasheet
            tempData =( ( (  (sht30_buf[0]*256) +sht30_buf[1]) *175   )/65535.0  -45  );
            humData =  ( ( (sht30_buf[3]*256) + (sht30_buf[4]) )*100/65535.0) ;
            ESP_LOGI("SHT30", "temp:%4.2f C   hum:%4.2f %%RH \r", tempData, humData); //℃打印出来是乱码
        }
        //SGP30数据测量及计算
        sgp30_IAQ_measure(&main_sgp30_sensor);
        //将值通过串口发送出去
        ESP_LOGI("SGP30", "TVOC: %d,  eCO2: %d\n",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);
        sensor_ui_set(tempData,humData,main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
