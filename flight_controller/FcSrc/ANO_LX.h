#ifndef __ANO_LX_H
#define __ANO_LX_H
//==引用
#include "McuConfig.h"

//==定义/声明

	
enum 
{
	ch_1_rol=0,//右，左右
	ch_2_pit,//右，上下
	ch_3_thr,//左，上下
	ch_4_yaw,//左，左右
	ch_5_aux1,//swc
	ch_6_aux2,//swd
	ch_7_aux3,//
	ch_8_aux4,
	ch_9_aux5,	
	ch_10_aux6,	
};

//0x40
typedef struct
{
	s16 ch_[10]; //	

}__attribute__ ((__packed__)) _rc_ch_st;

typedef union 
{
	u8 byte_data[20];
	_rc_ch_st st_data;
}_rc_ch_un;

//0x41
typedef struct
{
	s16 rol;
	s16 pit;
	s16 thr;
	s16 yaw_dps;
	s16 vel_x;
	s16 vel_y;
	s16 vel_z;

}__attribute__ ((__packed__)) _rt_tar_st;

typedef union 
{
	u8 byte_data[14];
	_rt_tar_st st_data;
}_rt_tar_un;

//0x0D
typedef struct
{
	u16 voltage_100;
	u16 current_100;

}__attribute__ ((__packed__)) _fc_bat_st;

typedef union 
{
	u8 byte_data[4];
	_fc_bat_st st_data;
}_fc_bat_un;

//0x03
typedef struct
{
	s16 rol_x100;
	s16 pit_x100;
	s16 yaw_x100;
	u8 state;
}__attribute__ ((__packed__)) _fc_att_st;

typedef union 
{
	u8 byte_data[7];
	_fc_att_st st_data;
}_fc_att_un;

//0x04
typedef struct
{
	s16 w_x10000;
	s16 x_x10000;
	s16 y_x10000;
	s16 z_x10000;
	u8 state;
}__attribute__ ((__packed__)) _fc_att_qua_st;

typedef union 
{
	u8 byte_data[9];
	_fc_att_qua_st st_data;
}_fc_att_qua_un;

//0x05
typedef struct
{
	s32 fus;
	s32 add;
	u8 state;
}__attribute__ ((__packed__)) _fc_alt_st;

typedef union 
{
	u8 byte_data[9];
	_fc_alt_st st_data;
}_fc_alt_un;

//0x07
typedef struct
{
	s16 vel_x;
	s16 vel_y;
	s16 vel_z;

}__attribute__ ((__packed__)) _fc_vel_st;

typedef union 
{
	u8 byte_data[6];
	_fc_vel_st st_data;
}_fc_vel_un;
//
typedef struct
{
	u16 pwm_m1;
	u16 pwm_m2;
	u16 pwm_m3;
	u16 pwm_m4;
	u16 pwm_m5;
	u16 pwm_m6;
	u16 pwm_m7;
	u16 pwm_m8;
}_pwm_st;

typedef struct
{
	s16 acc_x ;
	s16 acc_y ;
	s16 acc_z ;
	s16 gyr_x ;
	s16 gyr_y ;
	s16 gyr_z ;	
	s16 state ;
	
}_fc_acc_st;

typedef struct
{
	s16 mag[3];
	int32_t  alt_bar;
	s16 tmp;
	u8 bar_sta;
	u8 mag_sta;
	
}_fc_mag_st;

typedef struct
{
	s16 acc_x ;
	s16 acc_y ;
	s16 acc_z ;
	s16 yaw;
}_fc_kz_st;

//==数据声明
extern _fc_att_un fc_att;
extern _fc_att_qua_un fc_att_qua;
extern _fc_vel_un fc_vel;
extern _rt_tar_un rt_tar;
extern _fc_bat_un fc_bat;
extern _pwm_st pwm_to_esc;

extern _fc_alt_un fc_alt;//新建类型
extern _fc_acc_st fc_acc;
extern _fc_mag_st fc_mag;
extern _fc_kz_st fc;
//==函数声明
//static


//public
void Set_m_speed(s16 x,s16 y,s16 z,s16 yaw);
void ANO_LX_Task(void);

#endif

