/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��PWM_IN.c
 * ����    ��PWM���벶��      
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "pwmin.h"
#include "head.h"

u16  Rise[4],Drop[4];
u16  RC_Pwm_In[8];
u16  RC_Pwm_In_his[8];

u16  Rise1[4],Drop1[4];
u16  RC_Pwm_In1[8];
u16  RC_Pwm_In_his1[8];
/*====================================================================================================*/
/*====================================================================================================*
**���� : PWM_IN_Config
**���� : ����PWM���벶��
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/*
	TIM1_BRK_IRQn               = 24,    
  TIM1_UP_IRQn                = 25,    
  TIM1_TRG_COM_IRQn           = 26,     
  TIM1_CC_IRQn         
	*/
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

}
 

/*====================================================================================================*/
void PWM4_IN_Config(void)
{
	  GPIO_InitTypeDef         GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_ICInitTypeDef  TIM4_ICInitStructure;

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 //ʹ��TIM4ʱ��
 	  RCC_APB2PeriphClockCmd(RCC_GPIO_TIM4, ENABLE);  

	  GPIO_InitStructure.GPIO_Pin  = TIM4_CH1 | TIM4_CH2 | TIM4_CH3 | TIM4_CH4;             
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIO_TIM4,TIM4_CH1 | TIM4_CH2 | TIM4_CH3 | TIM4_CH4);		

	  //��ʼ����ʱ��4 TIM4	 
	  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                   //�趨�������Զ���װֵ 
	  TIM_TimeBaseStructure.TIM_Prescaler =71; 	                   //Ԥ��Ƶ��   
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //����ʱ�ӷָ�:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	  //��ʼ��TIM4���벶�����
	  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
		
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	  TIM_Cmd(TIM4,ENABLE ); 
		
	  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);        //��������ж� ,����CC1IE�����ж�	
	  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
			
}

void PWM1_IN_Config(void)
{
	  GPIO_InitTypeDef         GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_ICInitTypeDef  TIM1_ICInitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

	  GPIO_InitStructure.GPIO_Pin  = TIM1_CH1 | TIM1_CH2 | TIM1_CH3 | TIM1_CH4;             
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIO_TIM1,TIM1_CH1 | TIM1_CH2 | TIM1_CH3 | TIM1_CH4);		

	  //��ʼ����ʱ��4 TIM4	 
	  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                   //�趨�������Զ���װֵ 
	  TIM_TimeBaseStructure.TIM_Prescaler =71; 	                   //Ԥ��Ƶ��   
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //����ʱ�ӷָ�:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);              //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	  //��ʼ��TIM4���벶�����
	  TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //�����ز���
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	  TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
		TIM1_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
		
		TIM1_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //�����ز���
  	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //���������Ƶ,����Ƶ 
  	TIM1_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
  	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	
	  TIM_Cmd(TIM1,ENABLE ); 
		
	  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);        //��������ж� ,����CC1IE�����ж�	
	  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
			
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : TIM4_IRQHandler
**���� : TIM4�жϷ���
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void TIM4_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH1) == 1) 
			{
				  TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise[0]=TIM_GetCapture1(TIM4);
      }
			else 
			{
				  TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop[0]=TIM_GetCapture1(TIM4);
				  if(Rise[0]>Drop[0])  RC_Pwm_In[2] = 65535-Rise[0] + Drop[0];
					else 	               RC_Pwm_In[2] = Drop[0] - Rise[0];
      }			
		}	
	  
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH2) == 1) 
			{
				  TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise[1]=TIM_GetCapture2(TIM4);
      }
			else 
			{
				  TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop[1]=TIM_GetCapture2(TIM4);
				  if(Rise[1]>Drop[1])  RC_Pwm_In[3] = 65535-Rise[1] + Drop[1];
					else 	               RC_Pwm_In[3] = Drop[1] - Rise[1];
      }			
		}	
		
    if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)            //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH3) == 1) 
			{
				  TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise[2]=TIM_GetCapture3(TIM4);
      }
			else 
			{
				  TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop[2]=TIM_GetCapture3(TIM4);
				  if(Rise[2]>Drop[2]) RC_Pwm_In[1] = 65535-Rise[2] + Drop[2];
					else 	              RC_Pwm_In[1] = Drop[2] - Rise[2];
      }	 
		}	

    if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)            //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); //����жϱ�־λ
		  if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH4) == 1) 
			{
				  TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise[3]=TIM_GetCapture4(TIM4);
      }
			else 
			{
				  TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop[3]=TIM_GetCapture4(TIM4);
				  if(Rise[3]>Drop[3])  RC_Pwm_In[0] = 65535-Rise[3] + Drop[3];
					else 	               RC_Pwm_In[0] = Drop[3] - Rise[3];
      }	  
		}		
}



void TIM1_CC_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM1,TIM1_CH1) == 1) 
			{
				  TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise1[0]=TIM_GetCapture1(TIM1);
      }
			else 
			{
				  TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop1[0]=TIM_GetCapture1(TIM1);
				  if(Rise1[0]>Drop1[0])  RC_Pwm_In1[2] = 65535-Rise1[0] + Drop1[0];
					else 	               RC_Pwm_In1[2] = Drop1[0] - Rise1[0];
      }			
		}	
	  
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM1,TIM1_CH2) == 1) 
			{
				  TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise1[1]=TIM_GetCapture2(TIM1);
      }
			else 
			{
				  TIM_OC2PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop1[1]=TIM_GetCapture2(TIM1);
				  if(Rise1[1]>Drop1[1])  RC_Pwm_In1[3] = 65535-Rise1[1] + Drop1[1];
					else 	               RC_Pwm_In1[3] = Drop1[1] - Rise1[1];
      }			
		}	
		
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)            //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIO_TIM1,TIM1_CH3) == 1) 
			{
				  TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise1[2]=TIM_GetCapture3(TIM1);
      }
			else 
			{
				  TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop1[2]=TIM_GetCapture3(TIM1);
				  if(Rise1[2]>Drop1[2]) RC_Pwm_In1[1] = 65535-Rise1[2] + Drop1[2];
					else 	              RC_Pwm_In1[1] = Drop1[2] - Rise1[2];
      }	 
		}	

    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)            //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //����жϱ�־λ
		  if(GPIO_ReadInputDataBit(GPIO_TIM1,TIM1_CH4) == 1) 
			{
				  TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
          Rise1[3]=TIM_GetCapture4(TIM1);
      }
			else 
			{
				  TIM_OC4PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
          Drop1[3]=TIM_GetCapture4(TIM1);
				  if(Rise1[3]>Drop1[3])  RC_Pwm_In1[0] = 65535-Rise1[3] + Drop1[3];
					else 	               RC_Pwm_In1[0] = Drop1[3] - Rise1[3];
      }	  
		}		
}

#include "filter.h"
#define MED_NUM 8
void PWMIN_TO_RC(void)
{
Rc_Get_PWM.PITCH =Moving_Median(0,MED_NUM,LIMIT(RC_Pwm_In[1],1000,2000));
Rc_Get_PWM.ROLL =Moving_Median(1,MED_NUM,LIMIT(RC_Pwm_In[0],1000,2000));
Rc_Get_PWM.YAW =Moving_Median(2,MED_NUM,LIMIT(RC_Pwm_In[2],1000,2000));
Rc_Get_PWM.THROTTLE =Moving_Median(3,MED_NUM,LIMIT(RC_Pwm_In[3],1000,2000));	

Rc_Get_PWM.AUX1 =Moving_Median(4,MED_NUM,LIMIT(RC_Pwm_In1[2],1000,2000));
Rc_Get_PWM.RST =Moving_Median(5,MED_NUM,LIMIT(RC_Pwm_In1[1],1000,2000));
Rc_Get_PWM.HEIGHT_MODE =Moving_Median(6,MED_NUM,LIMIT(RC_Pwm_In1[0],1000,2000));
Rc_Get_PWM.POS_MODE =Moving_Median(7,MED_NUM,LIMIT(RC_Pwm_In1[3],1000,2000));

}