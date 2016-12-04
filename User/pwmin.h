#ifndef _PWM_IN_H_
#define _PWM_IN_H_
#include "stm32f10x.h"
#include "stm32f10x_tim.h"


extern u16  RC_Pwm_In[8];
extern u16  RC_Pwm_In_his[8];
void Nvic_Init(void);
void PWMIN_TO_RC(void);
void PWM4_IN_Config(void);
void PWM1_IN_Config(void);
#define GPIO_TIM4	     GPIOB
#define TIM4_CH1       GPIO_Pin_6
#define TIM4_CH2       GPIO_Pin_7
#define TIM4_CH3       GPIO_Pin_8
#define TIM4_CH4       GPIO_Pin_9
#define RCC_GPIO_TIM4  RCC_APB2Periph_GPIOB

#define GPIO_TIM1	     GPIOA
#define TIM1_CH1       GPIO_Pin_8
#define TIM1_CH2       GPIO_Pin_9
#define TIM1_CH3       GPIO_Pin_10
#define TIM1_CH4       GPIO_Pin_11
#define RCC_GPIO_TIM1  RCC_APB2Periph_GPIOA
#endif
