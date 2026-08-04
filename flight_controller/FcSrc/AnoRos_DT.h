#ifndef _ANOROS_DT_H_
#define	_ANOROS_DT_H_

//头文件
#include "SysConfig.h"

typedef struct
{
	u8 head;
	u8 addr;
	u8 ID;
	u8 dataLen;
	u8 dataBuf[256];
	u8 sc1;
	u8 sc2;
}__attribute__ ((__packed__)) AnoPTRosStruct;	 

typedef union 
{
	AnoPTRosStruct frame;
	u8 rawBytes[sizeof(AnoPTRosStruct)];
}__attribute__ ((__packed__)) AnoPTRosFrame;



//检测是否接收到数据
void GetRosData_Check_State(float dT_s);
//接受数据
void AnoRosDTRaspRecvOneByte(u8 data);
//发送数据
void AnoRosDT_SendData(void);


#endif



