#ifndef LV_FONTS_H_
#define LV_FONTS_H_

#include "src/lvgl.h"
#include "src/font/lv_font.h"

//写入字库命令
//python C:\esp-idf\esp-idf_v4.4\esp-idf\components\esptool_py\esptool\esptool.py --chip esp32s3 --port COM5 --baud 115200 write_flash -z 0xF00000 D:\font_harmony_sans_20_lv1_lv2.bin

extern const lv_font_t font_harmony_sans_20;

void lv_port_font_harmony_sans_20_load(const char *partition_label);

#endif
