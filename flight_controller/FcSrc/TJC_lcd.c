#include "TJC_lcd.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_Uart.h"
#include "User_Task.h"
#include "AnoDTRasp.h"
#include "Drv_Adc.h"
#include <stdio.h>

extern int takeoff_ready;
extern int Con_flag;
int mission_step_cun=0;
int TJC_Duoji=0;
int qr_cun=0;
extern int qr_num_cun;
extern int qr_num;
s16 x_t_cun=0,y_t_cun=0,z_t_cun=0,yaw_t_cun=0,x_o_cun=0,y_o_cun=0,z_o_cun=0,yaw_o_cun=0;

void LCD_show_step_cun(int step_in)
{
    mission_step_cun=step_in;
}
/**
  * @brief 发电压给串口屏
  * @Param 
  * @retval 
  **/

void Send_qua(void)
{
    int vel;
    u8 buf_qua[20];
    vel=fc_bat.st_data.voltage_100;
    sprintf(buf_qua,"qua.txt=\"%02d.%02dV\"",vel/100,vel%100);//qua.txt="00.0"
    buf_qua[16]=0XFF;
    buf_qua[17]=0XFF;
    buf_qua[18]=0XFF;
    DrvUart1SendBuf(buf_qua,19);
}
/**
  * @brief 发坐标给串口屏
  * @Param 
  * @retval 
  **/
void Send_tf(void)
{
    u8 buf_tfx[20],buf_tfy[20],buf_tfz[20];
    
    sprintf(buf_tfx,"tf_x.txt=\"%4d\"",rosData.loc[0]);
    sprintf(buf_tfy,"tf_y.txt=\"%4d\"",rosData.loc[1]);
    sprintf(buf_tfz,"tf_z.txt=\"%4d\"",fc_alt.st_data.add);

    buf_tfx[15]=0XFF;
    buf_tfx[16]=0XFF;
    buf_tfx[17]=0XFF;
    buf_tfy[15]=0XFF;
    buf_tfy[16]=0XFF;
    buf_tfy[17]=0XFF;
    buf_tfz[15]=0XFF;
    buf_tfz[16]=0XFF;
    buf_tfz[17]=0XFF;
    
    
    DrvUart1SendBuf(buf_tfx,18);
    DrvUart1SendBuf(buf_tfy,18);
    DrvUart1SendBuf(buf_tfz,18);

    
    
}

/**
  * @brief 发已录入的航点给串口屏
  * @Param 
  * @retval 
  **/
void Send_route_now(u8 tf,s16 x,s16 y,s16 z,s16 w)
{
    u8 buf_routex_now[25],buf_routey_now[25],buf_routez_now[25],buf_routew_now[25];
    if(tf>=0XA0&&tf<=0XAF)
    {
        sprintf(buf_routex_now,"page4.%2x_x.txt=\"%4d\"",tf,x);
        sprintf(buf_routey_now,"page4.%2x_y.txt=\"%4d\"",tf,y);
        sprintf(buf_routez_now,"page4.%2x_z.txt=\"%4d\"",tf,z);
        sprintf(buf_routew_now,"page4.%2x_w.txt=\"%4d\"",tf,w);
    }
    else if(tf>=0XB0&&tf<=0XBF)
    {
        sprintf(buf_routex_now,"page5.%2x_x.txt=\"%4d\"",tf,x);
        sprintf(buf_routey_now,"page5.%2x_y.txt=\"%4d\"",tf,y);
        sprintf(buf_routez_now,"page5.%2x_z.txt=\"%4d\"",tf,z);
        sprintf(buf_routew_now,"page5.%2x_w.txt=\"%4d\"",tf,w);
    }
    buf_routex_now[21]=0XFF;
    buf_routex_now[22]=0XFF;
    buf_routex_now[23]=0XFF;
    buf_routey_now[21]=0XFF;
    buf_routey_now[22]=0XFF;
    buf_routey_now[23]=0XFF;
    buf_routez_now[21]=0XFF;
    buf_routez_now[22]=0XFF;
    buf_routez_now[23]=0XFF;
    buf_routew_now[21]=0XFF;
    buf_routew_now[22]=0XFF;
    buf_routew_now[23]=0XFF;
    DrvUart1SendBuf(buf_routex_now,24);
    DrvUart1SendBuf(buf_routey_now,24);

}

/**
  * @brief 发速度给串口屏
  * @Param flag=0为发实时速度，flag=1为发目标速度
  * @retval 
  **/
void Send_speed_cun(s16 x,s16 y,s16 z,s16 yaw,u8 flag)
{
    if(flag==0)
    {
        x_t_cun=x;
        y_t_cun=y;
        z_t_cun=z;
        yaw_t_cun=yaw;
    }else if(flag==1)
    {
        x_o_cun=x;
        y_o_cun=y;
        z_o_cun=z;
        yaw_o_cun=yaw;
    }
    
    
}
void Send_speed(s16 x,s16 y,s16 z,s16 yaw,u8 flag)
{
    u8 buf_st0[20],buf_st1[20],buf_st2[20],buf_st3[20];
    if(flag==0)
    {
        sprintf(buf_st0,"st0.txt=\"%+.5d\"",x);
        sprintf(buf_st1,"st1.txt=\"%+.5d\"",y);
        sprintf(buf_st2,"st2.txt=\"%+.5d\"",z);
        sprintf(buf_st3,"st3.txt=\"%+.5d\"",yaw);
    }else if(flag==1)
    {
        sprintf(buf_st0,"so0.txt=\"%+.5d\"",x);
        sprintf(buf_st1,"so1.txt=\"%+.5d\"",y);
        sprintf(buf_st2,"so2.txt=\"%+.5d\"",z);
        sprintf(buf_st3,"so3.txt=\"%+.5d\"",yaw);
    }
    
    buf_st0[16]=0xff;
    buf_st0[17]=0xff;
    buf_st0[18]=0xff;
    buf_st1[16]=0xff;
    buf_st1[17]=0xff;
    buf_st1[18]=0xff;
    buf_st2[16]=0xff;
    buf_st2[17]=0xff;
    buf_st2[18]=0xff;
    buf_st3[16]=0xff;
    buf_st3[17]=0xff;
    buf_st3[18]=0xff;
    DrvUart1SendBuf(buf_st0,19);
    DrvUart1SendBuf(buf_st1,19);
    DrvUart1SendBuf(buf_st2,19);
    DrvUart1SendBuf(buf_st3,19);
    
}

/**
  * @brief 发送速度数据来源给串口屏幕
  * @Param 
  * @retval 
  **/

void Send_speed_source(u8 a)
{
    u8 buf_[28];
    if(a==0)
    {
        sprintf(buf_,"page0.b1.txt=\"lidar\"");
        buf_[20]=0xFF;
        buf_[21]=0xFF;
        buf_[22]=0xFF;
        DrvUart1SendBuf(buf_,23);
    }else if(a==1)
    {
        sprintf(buf_,"page0.b1.txt=\"opt flow\"");
        buf_[23]=0xFF;
        buf_[24]=0xFF;
        buf_[25]=0xFF;
        DrvUart1SendBuf(buf_,26);
    }else
    {
        sprintf(buf_,"page0.b1.txt=\"error\"");
        buf_[20]=0xFF;
        buf_[21]=0xFF;
        buf_[22]=0xFF;
        DrvUart1SendBuf(buf_,23);
    }
    
}

void Send_qr_cun(int in)
{
    
    qr_cun=in;
    
}
void Send_qr(void)
{
    u8 buf_[24];
    sprintf(buf_,"page1.qr1.txt=\"%2d\"",qr_cun);
    buf_[18]=0xFF;
    buf_[19]=0xFF;
    buf_[20]=0xFF;
    DrvUart1SendBuf(buf_,21);
}
void Send_qr_t(int in)
{
    u8 buf_[24];
    sprintf(buf_,"page1.qr2.txt=\"%2d\"",in);
    buf_[18]=0xFF;
    buf_[19]=0xFF;
    buf_[20]=0xFF;
    DrvUart1SendBuf(buf_,21);
}
/**
  * @brief 串口屏接收一帧
  * @Param 
  * @retval 
  **/

void MY_TJC_GetOneByte(uint8_t data)
{
    static u8 _data_len = 0, _data_cnt = 0;
    static u8 rxstate = 0;
    static uint8_t _datatemp2[50];

    if (rxstate == 0 && data == 0xAA)//帧头
    {
        
        rxstate = 1;
        _datatemp2[0] = data;
        _data_cnt = 0;
    }
    else if (rxstate == 1)//长度
    {
        _data_len=data;
        _datatemp2[1] = data;
        rxstate = 2;
    }
    else if (rxstate == 2 && _data_len > 0)//数据
    {
        _data_len--;
        _datatemp2[2 + _data_cnt++] = data;
        if (_data_len == 0)
            rxstate = 3;
    }
    else if (rxstate == 3)//校验
    {
        
        rxstate = 4;
        _datatemp2[2 + _data_cnt++] = data;
    }
    else if (rxstate == 4&& data == 0xFF)//帧尾
    {
        
        rxstate = 0;
        _datatemp2[2 + _data_cnt++] = data;
        MY_TJC_Data(_datatemp2, _data_cnt + 2); //
    }
    else
    {
        rxstate = 0;
    }
}

/**
  * @brief 串口屏数据包解析
  * @Param 
  * @retval 
  **/

static void MY_TJC_Data(uint8_t *data, uint8_t len)
{
    
    u8 the_speed_source;
    u8 check_sum1 = 0;
    
    for (u8 i = 1; i < len - 2; i++)
    {
        check_sum1 += *(data + i);
        
    }
    if(check_sum1 != *(data+len-2))
        return;
    
    if (*(data + 1) == 0X01) //
    {
        if (*(data + 2) == 0x01) //各种速度清零
        {
            Send_speed_cun(0,0,0,0,0);
            Send_speed_cun(0,0,0,0,1);
            Set_m_speed_now(0,0,0);
            Set_m_speed(0,0,0,0);
        }else if (*(data + 2) == 0x02) //速度来源改变
        {
            the_speed_source=Set_source_speed(-1);
            Send_speed_source(the_speed_source);
        }else if (*(data + 2) == 0x03) //
        {
            
        }else if(*(data + 2) >= 0xA0&&*(data + 2) <= 0xBF)//航点录入
        {
            test1();
            AnoPTv7SendRouteData((*(data + 2)&0X0F)+((((*(data + 2))-0XA0)>>4))*6,*(data + 2));
        }else if(*(data+2)==0X10)
        {
            TJC_Duoji=0;
        }else if(*(data+2)==0X11)
        {
            TJC_Duoji=1;
        }else if(*(data+2)==0X12)
        {
            TJC_Duoji=2;
        }else if(*(data+2)==0X13)
        {
            TJC_Duoji=3;
        }
    }
    else if(*(data + 1) == 0X04)
    {
        if(*(data + 2) == 0x23&&*(data + 3) == 0x04&&*(data + 4) == 0x07&&*(data + 5) == 0x31)//起飞
        {
            takeoff_ready=1;
        }else if(*(data + 2) == 0x31&&*(data + 3) == 0x07&&*(data + 4) == 0x04&&*(data + 5) == 0x23)//重置起飞
        {
            takeoff_ready=0;
        }else if(*(data + 2) == 0x23&&*(data + 3) == 0x04&&*(data + 4) == 0x07&&*(data + 5) == 0x32)//起飞
        {
            takeoff_ready=2;
        }
        else if(*(data + 2) == 0x23&&*(data + 3) == 0x01&&*(data + 4) == 0x01&&*(data + 5) == 0x01)//
        {
            qr_num_cun=qr_num;
            Send_qr_t(qr_num_cun);
            
        }
    }
}

void TJC_Send(void)//50ms执行一次，用来发送循环发送的显示项
{
    static int TJC_cnt=0;
    TJC_cnt++;
    if(TJC_cnt%10==1)
    {
        Send_qua();
        Send_tf();
        
    }
    if(TJC_cnt%10==3)
    {

        Send_speed(x_t_cun,y_t_cun,z_t_cun,mission_step_cun,0);
        Send_speed(x_o_cun,y_o_cun,z_o_cun,yaw_o_cun,1);
    }
    
    
    if(TJC_cnt%2==0)
    {
        
        Send_qr();
    }
    
    if(TJC_cnt>=49999)
    {
        TJC_cnt=0;
    }
}

void test1(void)
{
    u8 test1_buf[20];
    sprintf(test1_buf,"qua.bco=1024");
    test1_buf[12]=0XFF;
    test1_buf[13]=0XFF;
    test1_buf[14]=0XFF;
    DrvUart1SendBuf(test1_buf,15);
}

void Send_end(void)
{
    u8 end_buf[3]={0XFF,0XFF,0XFF};
    DrvUart1SendBuf(end_buf,3);
}
