/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：定时器驱动
**********************************************************************************/

#include "Drv_Timer.h"
#include "Drv_led.h"

#define SYS_TIMx_IRQn TIM7_IRQn
#define SYS_TIMx TIM7
#define SYS_RCC_TIMx RCC_APB1Periph_TIM7

void TIM_CONF(u16 period_ms) //APB1  84M
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);

    TIM_DeInit(SYS_TIMx);

    /* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period = period_ms;

    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 时钟预分频数为 */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数

    TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);

    TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(SYS_TIMx, ENABLE);

    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, DISABLE); /*先关闭等待使用*/
}

void TIM4_PWM_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 使能TIM4和GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // 配置GPIOD Pin12为复用功能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 将PD12连接到TIM4的通道1
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

    // 定时器TIM4初始化
    TIM_TimeBaseStructure.TIM_Period = 20000-1; // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1; // 不预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);

    // 使能TIM4
    TIM_Cmd(TIM4, ENABLE);
}

void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = SYS_TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DrvTimerFcInit(void)
{
    TIM_CONF(1000);
    TIM4_PWM_Init();
    TIM_NVIC();

    /* TIM7 重新开时钟，开始计时 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
