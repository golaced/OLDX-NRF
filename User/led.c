#include "led.h"
#include "delay.h"
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0	;

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOB*/
  	GPIO_Init(GPIOA, &GPIO_InitStructure);		 
	

	/* �ر�����led��	*/
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