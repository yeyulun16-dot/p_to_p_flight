#include "Bluetooth.h"
#include "User_Task.h"
#include "TJC_lcd.h"

#define TIME_W 20//1秒没接收到蓝牙信息视为断开将速度设为0一次
#define V_he 20
#define V_fen 14

s16 b_speed_x=0,b_speed_y=0;
u8 b_cnt=0,b_cun=0;


void Bluetooth_GetOneByte(uint8_t data)
{
    static u8 rxstate = 0;
    static uint8_t _datatemp2[10];
    
    if (rxstate == 0 && data == 0xAA)
    {
        rxstate = 1;
        _datatemp2[0] = data;
    }
    else if (rxstate == 1)
    {
        rxstate = 2;
        _datatemp2[1] = data;
    }
    else if (rxstate == 2&& (data > 0xF0))
    {
        rxstate = 0;
        _datatemp2[2] = data;
        //		DT_data_cnt = _data_cnt+5;
        //
        Bluetooth_Data(_datatemp2, 3); //
    }
    else
    {
        rxstate = 0;
    }
    
}

static void Bluetooth_Data(uint8_t *data, uint8_t len)
{
    
    
    
	if((*(data+2)-*(data+1))%16!=0)
        return;
	
	
    switch(*(data+1))
    {
        case(0XA1):{b_speed_x=0;b_speed_y=0;b_cnt++;break;}//原点
        case(0XA2):{b_speed_x=V_he;b_speed_y=0;b_cnt++;break;}//上
        case(0XA3):{b_speed_x=V_fen;b_speed_y=-V_fen;b_cnt++;break;}//右上
        case(0XA4):{b_speed_x=0;b_speed_y=-V_he;b_cnt++;break;}//右
        case(0XA5):{b_speed_x=-V_fen;b_speed_y=-V_fen;b_cnt++;break;}//右下
        case(0XA6):{b_speed_x=-V_he;b_speed_y=0;b_cnt++;break;}//下
        case(0XA7):{b_speed_x=-V_fen;b_speed_y=V_fen;b_cnt++;break;}//左下
        case(0XA8):{b_speed_x=0;b_speed_y=V_he;b_cnt++;break;}//左
        case(0XA9):{b_speed_x=V_fen;b_speed_y=V_fen;b_cnt++;break;}//左上
        case(0XB1):{}//up
        case(0XB2):{}//down
    }
    
}


void Bluetooth_speed_check(void)
{
//    static u8 wait_time=0;
//    if(b_cun!=b_cnt)
//    {
//        b_cun=b_cnt;
//        Set_m_speed(b_speed_x,b_speed_y,0,0);
//        wait_time=0;
//        
//    }else
//    {
//        if(wait_time<TIME_W)
//        {
//            wait_time++;
//        }
//        if(wait_time==TIME_W)
//        {
//            Set_m_speed(0,0,0,0);
//            Send_speed_cun(0,0,0,0,1);
//            wait_time++;
//        }else if(wait_time>TIME_W)
//        {
//            
//        }
//    }
}




