## 环境要求
+ ESP-IDF 4.4.6  
+ ESP32S3 N16R8
+ SD卡
+ ST7796 显示器,分辨率为320*480
+ GT911电容触摸
+ LVGL V8.3
+ MAX98357模块(音频部分需要)

## 引脚设置
引脚的相关设置在main文件夹下的board.h中定义  
未来尝试和外设封装在一起做成板级驱动（画饼）
__引脚号__ | __用途__|__备注__
----------|---------|-------
11|SD_CMD_IO<br>SD_MOSI_IO|上拉
12|SD_CLK_IO<br>SD_CLK_IO|上拉
13|SD_D0_IO<br>SD_MISO_IO|上拉
38|SD_CS_IO|上拉
35|PSRAM和FLASH占用
36|PSRAM和FLASH占用?UART_RX
37|PSRAM和FLASH占用?UART_TX
44|UART_RX
43|UART_TX
41|TOUCH_I2C_SCL
42|TOUCH_I2C_SDA
39|CONFIG_GT911_RST_PIN
40|CONFIG_GT911_INT_PIN
45|AUDIO_I2S_PIN_BCK
1|SCEEEN_DATA0_IO
2|SCEEEN_DATA1_IO
7|SCEEEN_DATA2_IO
8|SCEEEN_DATA3_IO
3|SCEEEN_DATA4_IO
18|SCEEEN_DATA5_IO
17|SCEEEN_DATA6_IO
16|SCEEEN_DATA7_IO
15|SCEEEN_PCLK_IO
10|SCEEEN_CS_IO
9|SCEEEN_DC_IO
14|SCEEEN_RST_IO
4|SCEEEN_BK_LIGHT_IO
19|USB_D-_IO|未使用
20|USB_D+_IO|未使用
0|RESET|按下复位
5|BAT_ADC_IO|未使用
6|光敏电阻|ADC1_CH5
21|未使用
46|AUDIO_I2S_PIN_DATA
47|AUDIO_I2S_PIN_WS
48|未使用


---  

## 烧录说明
由于本项目无factory分区，因此使用IDF烧录可能无法成功烧录，请在ESP32Project编译好后使用烧录软件将ESP32Project/build生成的文件按如下地址烧录  
或是在[release](https://github.com/herexiong/ESP32_ToDoList/releases)下载文件  
烧录完成后需要在IDF下再烧录一遍，否则无法启动 
[烧录软件](./resource_file/tool/flash_download_tool_3.9.3_0.zip)
__分区名__ | __地址__|__文件地址__|__备注__  
----------|---------|-------|-------
bootload | 0x1000 | ./ESP32Project/build/bootloader/bootloader.bin
partition_table | 0x8000 | ./ESP32Project/build/partition_table/partition-table.bin
ota_0 | 0x20000 | ./ESP32Project/build/ESP32_ToDoList.bin
font_hs20|0xF00000| ./ESP32Project/resource_file/font_harmony_sans_20_lv1_lv2.bin| 字体文件，烧录过一次后，更新无需烧录，在工程目录下，release下没有