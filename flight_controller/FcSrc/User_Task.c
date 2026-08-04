#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h"
#include "Drv_AnoOf.h"
#include "LX_FC_EXT_Sensor.h"
#include "Others.h"
#include "AnoDTRasp.h"
#include "TJC_lcd.h"

int takeoff_if=0;
int takeoff_ready=0;
extern int Con_flag;
extern int my_fly_flag;
extern int Ano_Duoji;
extern int TJC_Duoji;
extern int qr_num;

int run_target_duoji=2350;

void Duoji_Set(int place)
{
    switch(place)
    {
        case 0:
        {
            TIM_SetCompare1(TIM4, 600);
        }
        break;
        case 1:
        {
            TIM_SetCompare1(TIM4, 1000);
        }
        break;
        case 2:
        {
            TIM_SetCompare1(TIM4, 1400);
        }
        break;
        case 3:
        {
            TIM_SetCompare1(TIM4, 1800);
        }
        break;
    }
}
void Duoji_top_set(void)
{
    if(takeoff_ready)
    {
        Duoji_Set(Ano_Duoji);
    }else
    {
        Duoji_Set(TJC_Duoji);
    }
}

/**
  * @brief 任务函数
  * @Param 
  * @retval 
  **/


void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_down_f = 1,one_key_land_f = 1,one_key_mission_f=0,mission_step;
    LCD_show_step_cun(mission_step);
    //判断有遥控信号才执行
    if (rc_in.no_signal == 0)
    {
        //判断第6通道拨杆位置 1700<CH_6<2200
        if(rc_in.rc_ch.st_data.ch_[ch_6_aux2]>1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<2200)
        {
//            Duoji_Set(1);//600-1000-1400-1800
            //还没有执行
            if(takeoff_ready)
            {
                if(one_key_mission_f ==0)
                {
                    //标记已经执行
                    one_key_mission_f = 1;
                    //开始流程
                    mission_step = 1;
                }
            }
            else
            {
                //复位标记，以便再次执行
                one_key_mission_f = 0;
            }
            
        }
        else
        {
            //复位标记，以便再次执行
            one_key_mission_f = 0;
        }
        if(one_key_mission_f==1)
        {
            static u16 time_dly_cnt_ms=0;
            //
            switch(mission_step)
            {
                case 0:
                {
                    //reset
                    time_dly_cnt_ms = 0;
                    
                }
                break;
                case 1:
                {
                    my_fly_flag=1;
                    if(Con_flag)
                    {
                        mission_step += LX_Change_Mode(2);
                    }
                    
                    
                }
                break;
                case 2:
                {
                    //解锁
                    mission_step += FC_Unlock();
                    
                }
                break;
                case 3:
                {
                    //等2秒
                    if(time_dly_cnt_ms<2000)
                    {
                        time_dly_cnt_ms+=20;//ms
                    }
                    else
                    {
                        time_dly_cnt_ms = 0;
                        mission_step += 1;
                    }
                }
                break;
                case 4:
                {
                    takeoff_if=1;
                    mission_step += 1;
                }
                break;
                case 5:
                {
                    if(rosData.acc[2]<=-98 && rosData.acc[2]>=-102) 
                    {
                        mission_step += FC_Lock();
                        takeoff_if=0;
                        
                    }
                }
                break;
                case 6:
                {
                    takeoff_ready=0;
//                    if(rosData.acc[2]<=-78 && rosData.acc[2]>=-82) 
//                    {
//                        mission_step += FC_Unlock();
//                    }
                }
                break;
                case 7:
                {
                    mission_step=3;
                }
                break;
                
            }
            
        }
        else
        {
            mission_step=0;
            
        }
//        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
//        {
//            if(takeoff_ready)
//            {
//                //还没有执行
//                if (one_key_takeoff_f == 0)
//                    
//                {
//                    //标记已经执行
//                    one_key_takeoff_f =1;
//                }
//            }
//        }
//        else
//        {
//            //复位标记，以便再次执行
//            one_key_takeoff_f = 0;
//        }
        
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            takeoff_if=0;

            if (one_key_down_f == 0)
            {
                
            }else if(one_key_land_f==0)
            {
                
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
            one_key_down_f = 0;
        }
	}
    ////////////////////////////////////////////////////////////////////////
}
