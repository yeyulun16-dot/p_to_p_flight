#include "AnoDTRasp.h"
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"
#include "Drv_led.h"
#include "AnoRos_DT.h"
#include "LX_FC_EXT_Sensor.h"
#include "User_Task.h"
#include "ANO_LX.h"
#include "TJC_lcd.h"

static void anoDTRaspDataFrameAnl(void);
static void anoDTRaspSendCheck(void);
int Con_flag=0;
int my_fly_flag=0;
int Ano_Duoji=0;
int qr_num=0;
int qr_num_cun=0;
//接收数据缓冲
static myTAnoPTv7Frame rxFrame;
//发送数据缓冲
#define ANOPTV8TXBUFNUM	10
static myTPTv7TxFrame txFrameBuf[ANOPTV8TXBUFNUM];
static u8 txBufWAddr = 0;
static u8 txBufRAddr = 0;

ROS_DATA_ST rosData;//接受数据结构体

/*
 * 点到点飞行速度指令看门狗。
 * 正常情况下 Orange Pi 会以 50Hz 发送 0x31 帧；若串口、ROS 或定位
 * 链路中断，不允许飞控无限保持最后一条非零速度指令。
 */
#define ROS_VELOCITY_TIMEOUT_MS 300U
static u16 ros_velocity_age_ms = 0U;
static u8 ros_velocity_seen = 0U;
static u8 ros_velocity_timeout_latched = 0U;

//
void AnoDTRaspRunTask1Ms(void)
{
    static u16 tmp_cnt[2];

    if (ros_velocity_seen && !ros_velocity_timeout_latched)
    {
        if (ros_velocity_age_ms < ROS_VELOCITY_TIMEOUT_MS)
        {
            ros_velocity_age_ms++;
        }
        if (ros_velocity_age_ms >= ROS_VELOCITY_TIMEOUT_MS)
        {
            Set_m_speed(0, 0, 0, 0);
            ros_velocity_timeout_latched = 1U;
        }
    }
    //计5ms
    tmp_cnt[0]++;
    tmp_cnt[0] %= 5;
    if (tmp_cnt[0] == 0)
    {
        AnoPTv7SendALT(fc_alt.st_data.add);
//		AnoPTv7SendVEL(fc_vel.st_data.vel_x,fc_vel.st_data.vel_y,fc_vel.st_data.vel_z);
//		AnoPTv7SendMAG(fc_mag.mag[0],fc_mag.mag[1],fc_mag.mag[2],fc_mag.tmp);
//		AnoPTv7SendRC(
//						rc_in.rc_ch.st_data.ch_[ch_1_rol],
//						rc_in.rc_ch.st_data.ch_[ch_2_pit],
//						rc_in.rc_ch.st_data.ch_[ch_3_thr],
//						rc_in.rc_ch.st_data.ch_[ch_4_yaw],
//						rc_in.rc_ch.st_data.ch_[ch_5_aux1],
//						rc_in.rc_ch.st_data.ch_[ch_6_aux2],
//						rc_in.rc_ch.st_data.ch_[ch_7_aux3],
//						rc_in.rc_ch.st_data.ch_[ch_8_aux4],
//						rc_in.rc_ch.st_data.ch_[ch_9_aux5],
//						rc_in.rc_ch.st_data.ch_[ch_10_aux6]
//					);
        //计10ms
        tmp_cnt[1]++;
        tmp_cnt[1] %= 2;
        if (tmp_cnt[1] == 0)
        {
//			AnoPTv7SendGPS(100000000,-100000000,100000000);
			AnoPTv7SendUserData(0,my_fly_flag,qr_num_cun,-303,303,3333);
            
        }
    }
    
//    AnoPTv7SendIMU(
//                    fc_acc.acc_x,fc_acc.acc_y,fc_acc.acc_z,
//                    fc_acc.gyr_x,fc_acc.gyr_y,fc_acc.gyr_z,
//                    fc_att_qua.st_data.w_x10000,
//                    fc_att_qua.st_data.x_x10000,
//                    fc_att_qua.st_data.y_x10000,
//                    fc_att_qua.st_data.z_x10000
//                );
    //检查是否有数据需要发送
    anoDTRaspSendCheck();
}
//
void AnoDTRaspRecvOneByte(u8 data)
{

    static u16 _recv_cnt = 0;
    static u16 _data_cnt = 0;
    static u8 rxstate = 0;

    //判断帧头是否满足匿名协议的0xAB
    if (rxstate == 0 && data == 0xAA)
    {
        _recv_cnt = 0;
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //数据地址字节
    else if (rxstate == 1)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //帧ID
    else if (rxstate == 2)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //接收数据长度
    else if (rxstate == 3)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxFrame.frame.dataLen = data;
        _data_cnt = 0;
        rxstate ++;
    }
    //接收数据
    else if (rxstate == 4)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        _data_cnt++;
        if(_data_cnt >= rxFrame.frame.dataLen)
        {
            rxstate ++;
        }
    }
    //接收SC1
    else  if (rxstate == 5)
    {
        rxFrame.frame.sc1 = data;
        rxstate ++;
    }
    //接收SC2
    else  if (rxstate == 6)
    {
        rxFrame.frame.sc2 = data;
        rxstate = 0;
        //所有数据接收完毕，进行解析
        anoDTRaspDataFrameAnl();
    }
    else
    {
        rxstate = 0;
    }
}


static void anoDTRaspDataFrameAnl(void)
{
    u8 check_sum1 = 0, check_sum2 = 0;
    int zancun=0;
    //根据收到的数据计算校验字节1和2
    for (u16 i = 0; i < (rxFrame.frame.dataLen + 4); i++)
    {
        check_sum1 += rxFrame.rawBytes[i];
        check_sum2 += check_sum1;
    }
    //计算出的校验字节和收到的校验字节做对比，完全一致代表本帧数据合法，不一致则跳出解析函数
    if ((check_sum1 != rxFrame.frame.sc1) || (check_sum2 != rxFrame.frame.sc2)) //判断sum校验
        return;
    /*******************************************/
    switch(rxFrame.frame.ID)
    {
        case 0xA2:
        {
            Con_flag=(s8)*((s8*)(rxFrame.frame.dataBuf + 0));
            Ano_Duoji=(s8)*((s8*)(rxFrame.frame.dataBuf + 2));
            zancun=(s16)*((s16*)(rxFrame.frame.dataBuf + 7));
            if(zancun!=0)
                qr_num=zancun;
            Send_qr_cun(qr_num);
        }
        break;
        case 0x31://速度控制帧
        {
            if (rxFrame.frame.dataLen >= 8U)
            {
                rosData.acc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
                rosData.acc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
                rosData.acc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
                rosData.yaw    = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
                Set_m_speed(rosData.acc[0],rosData.acc[1],rosData.acc[2],rosData.yaw);
                ros_velocity_age_ms = 0U;
                ros_velocity_seen = 1U;
                ros_velocity_timeout_latched = 0U;
            }

        }
        break;
        case 0x66://任务结束/停止输出帧，data[0] == 0x06
        {
            if ((rxFrame.frame.dataLen >= 1U) && (rxFrame.frame.dataBuf[0] == 0x06U))
            {
                Set_m_speed(0, 0, 0, 0);
                takeoff_if = 0;
                takeoff_ready = 0;
                Con_flag = 0;
                ros_velocity_age_ms = ROS_VELOCITY_TIMEOUT_MS;
                ros_velocity_seen = 1U;
                ros_velocity_timeout_latched = 1U;
            }
        }
        break;
        case 0x67://点到点任务使能帧：0x01=准备，0x00=撤销
        {
            if (rxFrame.frame.dataLen >= 1U)
            {
                if (rxFrame.frame.dataBuf[0] == 0x01U)
                {
                    /* 仅准备外部任务；仍需遥控器 CH6 高位才会切模式并解锁。 */
                    Con_flag = 1;
                    takeoff_ready = 1;
                }
                else if (rxFrame.frame.dataBuf[0] == 0x00U)
                {
                    Set_m_speed(0, 0, 0, 0);
                    takeoff_if = 0;
                    takeoff_ready = 0;
                    Con_flag = 0;
                }
            }
        }
        break;
        case 0x32://位置与速度，t(原来指t265)
        {			
            
            rosData.tacc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
            rosData.tacc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
            rosData.tacc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
            
            rosData.tloc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
            rosData.tloc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 8));
            rosData.tloc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 10));
            
            Set_m_speed_now(rosData.tloc[0],rosData.tloc[1],rosData.tloc[2]);
            Send_speed_cun(rosData.tloc[0],rosData.tloc[1],rosData.tloc[2],0,0);
        }
        break;
        case 0x33://激光雷达位置坐标
        {
            rosData.loc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
            rosData.loc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
            rosData.loc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
        }break;
        case 0X63:
        {
            Send_route_now((s8)*((s8*)(rxFrame.frame.dataBuf+0)),(s16)*((s16*)(rxFrame.frame.dataBuf + 1)),(s16)*((s16*)(rxFrame.frame.dataBuf + 3)),(s16)*((s16*)(rxFrame.frame.dataBuf + 5)),(s16)*((s16*)(rxFrame.frame.dataBuf + 7)));
        }
        break;
        default:
            break;
    }
}


//
void AnoPTv7DrvSend(u8 linktype, u8 *buf, u16 len)
{
    DrvUart3SendBuf(buf,len);
}
//
//将需要发送的帧写入缓冲区
static void anoPTv7AddTxFrame(u8 linkType, myTAnoPTv7Frame *pFrame)
{
    if(txFrameBuf[txBufWAddr].cmd._isUsed)
        return;
    txFrameBuf[txBufWAddr].cmd._isUsed = 1;
    txFrameBuf[txBufWAddr].cmd._linkType = linkType;
    for(u16 i = 0; i < (pFrame->frame.dataLen + 6); i++)
        txFrameBuf[txBufWAddr].data.rawBytes[i] = pFrame->rawBytes[i];
    
    txFrameBuf[txBufWAddr].cmd._isUsed = 2;
    
    txBufWAddr++;
    if(txBufWAddr >= ANOPTV8TXBUFNUM)
        txBufWAddr = 0;
}
//
static void anoDTRaspSendCheck(void)
{
    //如果发送缓冲区有需要发送的数据，执行发送
    for(u8 i=0; i<ANOPTV8TXBUFNUM; i++)
    {
        if(txFrameBuf[txBufRAddr].cmd._isUsed == 2)
        {
            //数据写入完毕，调用发送
            AnoPTv7DrvSend(txFrameBuf[txBufRAddr].cmd._linkType, txFrameBuf[txBufRAddr].data.rawBytes, txFrameBuf[txBufRAddr].data.frame.dataLen+6);
            txFrameBuf[txBufRAddr].cmd._isUsed = 0;
        }
        txBufRAddr++;
        if(txBufRAddr >= ANOPTV8TXBUFNUM)
        txBufRAddr = 0;
    }
}

//发送速度数据
void AnoPTv7SendVEL(s16 vel_x,s16 vel_y,s16 vel_z)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x07;
    txFrame.frame.dataLen = 6;
    txFrame.frame.dataBuf[0] = BYTE0(vel_x);
    txFrame.frame.dataBuf[1] = BYTE1(vel_x);
    
    txFrame.frame.dataBuf[2] = BYTE0(vel_y);
    txFrame.frame.dataBuf[3] = BYTE1(vel_y);
    
    txFrame.frame.dataBuf[4] = BYTE0(vel_z);
    txFrame.frame.dataBuf[5] = BYTE1(vel_z);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送GPS数据
void AnoPTv7SendGPS(s32 GPS_LNG,s32 GPS_LAT,s32 GPS_ALT)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x30;
    txFrame.frame.dataLen = 12;
    txFrame.frame.dataBuf[0] = BYTE0(GPS_LNG);
    txFrame.frame.dataBuf[1] = BYTE1(GPS_LNG);
    txFrame.frame.dataBuf[2] = BYTE2(GPS_LNG);
    txFrame.frame.dataBuf[3] = BYTE3(GPS_LNG);
 
    txFrame.frame.dataBuf[4] = BYTE0(GPS_LAT);
    txFrame.frame.dataBuf[5] = BYTE1(GPS_LAT);
    txFrame.frame.dataBuf[6] = BYTE2(GPS_LAT);
    txFrame.frame.dataBuf[7] = BYTE3(GPS_LAT);
    
    txFrame.frame.dataBuf[8] = BYTE0(GPS_ALT);
    txFrame.frame.dataBuf[9] = BYTE1(GPS_ALT);
    txFrame.frame.dataBuf[10] = BYTE2(GPS_ALT);
    txFrame.frame.dataBuf[11] = BYTE3(GPS_ALT);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送imu数据与四元素
void AnoPTv7SendIMU(s16 accx,s16 accy,s16 accz,s16 gyr_x,s16 gyr_y,s16 gyr_z,s16 v0,s16 v1,s16 v2,s16 v3)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x01;
    txFrame.frame.dataLen = 20;
    txFrame.frame.dataBuf[0] = BYTE0(accx);
    txFrame.frame.dataBuf[1] = BYTE1(accx);

    txFrame.frame.dataBuf[2] = BYTE0(accy);
    txFrame.frame.dataBuf[3] = BYTE1(accy);

    txFrame.frame.dataBuf[4] = BYTE0(accz);
    txFrame.frame.dataBuf[5] = BYTE1(accz);

    txFrame.frame.dataBuf[6] = BYTE0(gyr_x);
    txFrame.frame.dataBuf[7] = BYTE1(gyr_x);

    txFrame.frame.dataBuf[8] = BYTE0(gyr_y);
    txFrame.frame.dataBuf[9] = BYTE1(gyr_y);

    txFrame.frame.dataBuf[10] = BYTE0(gyr_z);
    txFrame.frame.dataBuf[11] = BYTE1(gyr_z);
    
    txFrame.frame.dataBuf[12] = BYTE0(v0);
    txFrame.frame.dataBuf[13] = BYTE1(v0);
    
    txFrame.frame.dataBuf[14] = BYTE0(v1);
    txFrame.frame.dataBuf[15] = BYTE1(v1);
    
    txFrame.frame.dataBuf[16] = BYTE0(v2);
    txFrame.frame.dataBuf[17] = BYTE1(v2);
    
    txFrame.frame.dataBuf[18] = BYTE0(v3);
    txFrame.frame.dataBuf[19] = BYTE1(v3);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送用户数据
void AnoPTv7SendUserData(u8 d1,u8 d2,u8 d3,s16 d4,s16 d5,s16 d6)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0xF1;
    txFrame.frame.dataLen = 9;
    txFrame.frame.dataBuf[0] = d1;
    txFrame.frame.dataBuf[1] = d2;
    txFrame.frame.dataBuf[2] = d3;

    txFrame.frame.dataBuf[3] = BYTE0(d4);
    txFrame.frame.dataBuf[4] = BYTE1(d4);

    txFrame.frame.dataBuf[5] = BYTE0(d5);
    txFrame.frame.dataBuf[6] = BYTE1(d5);

    txFrame.frame.dataBuf[7] = BYTE0(d6);
    txFrame.frame.dataBuf[8] = BYTE1(d6);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
//发送航点数据
void AnoPTv7SendRouteData(u8 d1,u8 d2)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x62;
    txFrame.frame.dataLen = 2;
    txFrame.frame.dataBuf[0] = d1;
    txFrame.frame.dataBuf[1] = d2;
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
//发送罗盘数据
void AnoPTv7SendMAG(s16 mag_x,s16 mag_y,s16 mag_z,s16 tmp)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x02;
    txFrame.frame.dataLen = 8;
    txFrame.frame.dataBuf[0] = BYTE0(mag_x);
    txFrame.frame.dataBuf[1] = BYTE1(mag_x);
    
    txFrame.frame.dataBuf[2] = BYTE0(mag_y);
    txFrame.frame.dataBuf[3] = BYTE1(mag_y);
    
    txFrame.frame.dataBuf[4] = BYTE0(mag_z);
    txFrame.frame.dataBuf[5] = BYTE1(mag_z);
    
    txFrame.frame.dataBuf[6] = BYTE0(tmp);
    txFrame.frame.dataBuf[7] = BYTE1(tmp);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送高度数据
void AnoPTv7SendALT(s16 alt)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x05;
    txFrame.frame.dataLen = 2;
    txFrame.frame.dataBuf[0] = BYTE0(alt);
    txFrame.frame.dataBuf[1] = BYTE1(alt);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//发送遥控器数据
void AnoPTv7SendRC(u16 ch_1,u16 ch_2,u16 ch_3,u16 ch_4,u16 ch_5,u16 ch_6,u16 ch_7,u16 ch_8,u16 ch_9,u16 ch_10)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x40;
    txFrame.frame.dataLen = 20;
    txFrame.frame.dataBuf[0] = BYTE0(ch_1);
    txFrame.frame.dataBuf[1] = BYTE1(ch_1);

    txFrame.frame.dataBuf[2] = BYTE0(ch_2);
    txFrame.frame.dataBuf[3] = BYTE1(ch_2);

    txFrame.frame.dataBuf[4] = BYTE0(ch_3);
    txFrame.frame.dataBuf[5] = BYTE1(ch_3);

    txFrame.frame.dataBuf[6] = BYTE0(ch_4);
    txFrame.frame.dataBuf[7] = BYTE1(ch_4);

    txFrame.frame.dataBuf[8] = BYTE0(ch_5);
    txFrame.frame.dataBuf[9] = BYTE1(ch_5);

    txFrame.frame.dataBuf[10] = BYTE0(ch_6);
    txFrame.frame.dataBuf[11] = BYTE1(ch_6);

    txFrame.frame.dataBuf[12] = BYTE0(ch_7);
    txFrame.frame.dataBuf[13] = BYTE1(ch_7);
    
    txFrame.frame.dataBuf[14] = BYTE0(ch_8);
    txFrame.frame.dataBuf[15] = BYTE1(ch_8);
    
    txFrame.frame.dataBuf[16] = BYTE0(ch_9);
    txFrame.frame.dataBuf[17] = BYTE1(ch_9);
    
    txFrame.frame.dataBuf[18] = BYTE0(ch_10);
    txFrame.frame.dataBuf[19] = BYTE1(ch_10);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
