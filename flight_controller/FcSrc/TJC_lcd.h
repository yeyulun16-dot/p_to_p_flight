#ifndef __TJC_LCD_H__
#define __TJC_LCD_H__
#include "SysConfig.h"

void LCD_show_step_cun(int step_in);

void Send_qua(void);
void Send_tf(void);
void Send_route_now(u8 tf,s16 x,s16 y,s16 z,s16 w);
void Send_speed(s16 x,s16 y,s16 z,s16 yaw,u8 flag);
void Send_speed_source(u8 a);
void MY_TJC_GetOneByte(uint8_t data);
static void MY_TJC_Data(uint8_t *data, uint8_t len);
void Send_speed_cun(s16 x,s16 y,s16 z,s16 yaw,u8 flag);
void Send_qr_cun(int in);
void Send_qr(void);
void Send_qr_t(int in);
void TJC_Send(void);
void test1(void);
void Send_end(void);
#endif
