#include "head.h"
#include "pwmin.h"
int main(void)
{		ErrorStatus HSEStartUpStatus;
		//RCC reset
		RCC_DeInit();
		//�����ⲿʱ�� ��ִ�г�ʼ��
		RCC_HSEConfig(RCC_HSE_ON); 
		//�ȴ��ⲿʱ��׼����
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
		//����ʧ�� ������ȴ�
		while(HSEStartUpStatus == ERROR);
		//�����ڲ�����ʱ��
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//�ⲿʱ��Ϊ8M ���ﱶƵ��72M
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE); 
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
		//----------------------------- CLOSE HSI ---------------------------
		RCC_HSICmd(DISABLE);//�ر���`
	  Nvic_Init();
		delay_init(72);
		LED_GPIO_Config();
		Spi1_Init();		
		Nrf24l01_Init(MODEL_RX2,40);// α˫��  ������
		Nrf24l01_Check();
		delay_ms(2000);
		usart1_config();
		PWM4_IN_Config();
		PWM1_IN_Config();
		while(1){
			
			static u8 cnt[3];	
			if(cnt[0]++>5){cnt[0]=0;	
			RC_Send_Task();	//NRF ���� TO NRF-Gģ��
			}
			
			if(cnt[1]++>2){cnt[1]=0;	
			Nrf_Check_Event();//NRF ����
			PWMIN_TO_RC();//���ջ�ң�����˲�
			GOL_LINK_TASK();//����ͨѶ
			}
			delay_ms(5);
			
		}
}
/*********************************************END OF FILE**********************/

