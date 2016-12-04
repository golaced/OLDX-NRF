#include "led.h"
#include "delay.h"
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*选择要控制的GPIOC引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0	;

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		 
	

	/* 关闭所有led灯	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);	 
	delay_ms(200);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);	 
	delay_ms(200);
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);	 
	delay_ms(200);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);	 
}

void LED_STATE(u8 sel,u8 on)
{
switch(sel)
{
case RX_LED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
else
GPIO_SetBits(GPIOA,GPIO_Pin_0);
break;
case TX_LED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_1);
else
GPIO_SetBits(GPIOA,GPIO_Pin_1);
break;
}
}