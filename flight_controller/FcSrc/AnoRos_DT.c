#include "AnoRos_DT.h"
#include "Drv_Uart.h"
#include "LX_FC_EXT_Sensor.h"
#include "AnoDTRasp.h"


//数据拆分
//#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
//#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
//#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
//#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

//帧头
#define FRAME_HEAD		0xAA
//发送数据长度
#define DT_TX_BUFNUM 		64
//发送数据串口函数
#define DT_SENDPTR_UART 	DrvUart3SendBuf   

//加速度
#define ACC_RAW_X      (fc_acc.acc_x)//转换单位后的加速度
#define ACC_RAW_Y      (fc_acc.acc_y)
#define ACC_RAW_Z      (fc_acc.acc_z)
//陀螺仪
#define GYR_RAW_X      (fc_acc.gyr_x)//角度转弧度后的数据
#define GYR_RAW_Y      (fc_acc.gyr_y)
#define GYR_RAW_Z      (fc_acc.gyr_z)
//欧拉角
#define ANGLE_ROL      (fc_att.st_data.rol_x100)//欧拉角
#define ANGLE_PIT      (fc_att.st_data.pit_x100)
#define ANGLE_YAW      (fc_att.st_data.yaw_x100)
//四元素  
/*
注意：当没有四元素进行数据上传时，请按照w x y z 0 1 2 3 进行复制
	  当有四元素数据进行上传时，请替换即可
*/
#define	QUA0_10K (fc_att_qua.st_data.w_x10000)	
#define	QUA1_10K (fc_att_qua.st_data.x_x10000)
#define	QUA2_10K (fc_att_qua.st_data.y_x10000)
#define	QUA3_10K (fc_att_qua.st_data.z_x10000)

//运动速度
#define HCA_VEL_X      (fc_vel.st_data.vel_x)//速度在定点模式下才有数据
#define HCA_VEL_Y      (fc_vel.st_data.vel_y)
#define HCA_VEL_Z      (fc_vel.st_data.vel_z)

#define UserData1	(10)//s8
#define UserData2	(2)
#define UserData3	(3)
#define UserData4	(-1)//s16
#define UserData5	(2)
#define UserData6	(-30303)

#define GPS_LNG 100000000//(ext_sens.fc_gps.st_data.LNG)
#define GPS_LAT -100000000//(ext_sens.fc_gps.st_data.LAT)
#define GPS_ALT 100000000//(ext_sens.fc_gps.st_data.ALT_GPS)


//接受数据
static AnoPTRosFrame rxFrame;
//解析函数
static void anoRosDTRaspDataFrameAnl();



//循环发送数据临时缓冲
u8 otherDataTmp_[DT_TX_BUFNUM];

static float Ros_check_time_ms[1];
/*
功能：检测是否接收到Ros端发送的数据
*/
void GetRosData_Check_State(float dT_s)
{
	
	//数据检查
	if (Ros_check_time_ms[0] < 5)
	{
		Ros_check_time_ms[0]++;
		rosData.getRosDataFlag = 1;
	}
	else
	{
		rosData.getRosDataFlag = 0;
	}
}

/*
功能：接受ROS端发送的数据
*/
void AnoRosDTRaspRecvOneByte(u8 data)
{
    static u16 _recv_cnt = 0;
    static u16 _data_cnt = 0;
    static u8 rxstate = 0;

    //判断帧头是否满足匿名协议的0xAA
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
        anoRosDTRaspDataFrameAnl();
    }
    else
    {
        rxstate = 0;
    }
}

/*
功能：解析ROS端发送的数据
*/
static void anoRosDTRaspDataFrameAnl()
{
	 u8 check_sum1 = 0, check_sum2 = 0;
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
		case 0x31://速度控制帧
		{			
			rosData.acc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf));
			rosData.acc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
			rosData.acc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
			rosData.yaw    = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
			Ros_check_time_ms[0] = 0;
		}
		break;
        case 0x32://???
		{			
			rosData.tacc[0] = (s16)*((float*)(rxFrame.frame.dataBuf));
			rosData.tacc[1] = (s16)*((float*)(rxFrame.frame.dataBuf + 4));
			rosData.tacc[2] = (s16)*((float*)(rxFrame.frame.dataBuf + 8));
			
			rosData.tloc[0] = (s16)*((float*)(rxFrame.frame.dataBuf + 12));
			rosData.tloc[1] = (s16)*((float*)(rxFrame.frame.dataBuf + 16));
			rosData.tloc[2] = (s16)*((float*)(rxFrame.frame.dataBuf + 20));
			
			rosData.loc[0] = (s16)*((float*)(rxFrame.frame.dataBuf + 24));
			rosData.loc[1] = (s16)*((float*)(rxFrame.frame.dataBuf + 28));
			rosData.loc[2] = (s16)*((float*)(rxFrame.frame.dataBuf + 32));
			
			Ros_check_time_ms[0] = 0;
		}
		break;
		default:
			break;
	}
}
/*
功能：发送数据到ROS端
*/
void AnoRosDT_SendData(void)
{
	u16 _cnt = 0;
	s16 temp_data;
	s32 GPS_data;
	otherDataTmp_[_cnt++] = FRAME_HEAD;
	otherDataTmp_[_cnt++] = 0xFF;
	otherDataTmp_[_cnt++] = 0x01;
	otherDataTmp_[_cnt++] = 0;

	//加速度
	temp_data = (s16)ACC_RAW_X;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)ACC_RAW_Y;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//2
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)ACC_RAW_Z;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//4
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	//陀螺仪
	temp_data = (s16)GYR_RAW_X;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//6
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)GYR_RAW_Y;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//8
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)GYR_RAW_Z;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//10
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	//欧拉角
	temp_data = (s16)ANGLE_ROL;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//12
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)ANGLE_PIT;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//14
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)ANGLE_YAW;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//16
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	//四元素
	temp_data = (s16)QUA0_10K;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//18
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)QUA1_10K;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//20
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)QUA2_10K;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//22
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)QUA3_10K;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//24
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	//运动速度
	temp_data = (s16)HCA_VEL_X;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//26
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)HCA_VEL_Y;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//28
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)HCA_VEL_Z;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//30
	otherDataTmp_[_cnt++] = BYTE1(temp_data);//
	
    //用户数据
	otherDataTmp_[_cnt++] = UserData1;//32
	
	otherDataTmp_[_cnt++] = UserData2;//33
	
	otherDataTmp_[_cnt++] = UserData3;//34
	
	temp_data = (s16)UserData4;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//35
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)UserData5;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//37
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	
	temp_data = (s16)UserData6;
	otherDataTmp_[_cnt++] = BYTE0(temp_data);//39
	otherDataTmp_[_cnt++] = BYTE1(temp_data);
	//GPS数据
    GPS_data = (s32)GPS_LNG;
	otherDataTmp_[_cnt++] = BYTE0(GPS_data);//41
	otherDataTmp_[_cnt++] = BYTE1(GPS_data);
    otherDataTmp_[_cnt++] = BYTE2(GPS_data);
	otherDataTmp_[_cnt++] = BYTE3(GPS_data);
    
    GPS_data = (s32)GPS_LAT;
	otherDataTmp_[_cnt++] = BYTE0(GPS_data);
	otherDataTmp_[_cnt++] = BYTE1(GPS_data);
    otherDataTmp_[_cnt++] = BYTE2(GPS_data);
	otherDataTmp_[_cnt++] = BYTE3(GPS_data);
    
    GPS_data = (s32)GPS_ALT;
	otherDataTmp_[_cnt++] = BYTE0(GPS_data);
	otherDataTmp_[_cnt++] = BYTE1(GPS_data);
    otherDataTmp_[_cnt++] = BYTE2(GPS_data);
	otherDataTmp_[_cnt++] = BYTE3(GPS_data);
    
	otherDataTmp_[3] = _cnt-4;
	u8 check_sum1 = 0, check_sum2 = 0;
	for(u8 i=0;i<_cnt;i++)
	{
		check_sum1 += otherDataTmp_[i];
		check_sum2 += check_sum1;
	}
	otherDataTmp_[_cnt++] = check_sum1;
	otherDataTmp_[_cnt++] = check_sum2;
	
	DT_SENDPTR_UART(otherDataTmp_,_cnt);
}









