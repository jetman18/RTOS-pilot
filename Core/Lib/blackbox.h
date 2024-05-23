

#ifndef BLACKBOX_H
#define BLACKBOX_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "fatfs.h"

void black_box_init();
void black_box_pack_int(int val);
void black_box_pack_float(float val,uint8_t digit_after_point );
void black_box_pack_str(char *c);
void black_box_pack_char(char c);
void black_box_load();
void black_box_sync();
void black_box_close();
uint32_t black_box_get_file_size();
int black_box_read(char *file_name, char *bufferr,uint8_t len);
int black_box_pack(int param1, int  param2, int  param3,  int param4);

#ifdef __cplusplus
}
#endif

#endif

