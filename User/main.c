#include "head.h"
#include "pwmin.h"
int main(void)
{		ErrorStatus HSEStartUpStatus;
		//RCC reset
		RCC_DeInit();
		//开启外部时钟 并执行初始化
		RCC_HSEConfig(RCC_HSE_ON); 
		//等待外部时钟准备好
		HSEStartUpStatus = RCC_WaitForHSEStartUp();
		//启动失败 在这里等待
		while(HSEStartUpStatus == ERROR);
		//设置内部总线时钟
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//外部时钟为8M 这里倍频到72M
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE); 
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
		//----------------------------- CLOSE HSI ---------------------------
		RCC_HSICmd(DISABLE);//关闭内`
	  Nvic_Init();
		delay_init(72);
		LED_GPIO_Config();
		Spi1_Init();		
		Nrf24l01_Init(MODEL_RX2,40);// 伪双工  主接收
		Nrf24l01_Check();
		delay_ms(2000);
		usart1_config();
		PWM4_IN_Config();
		PWM1_IN_Config();
		while(1){
			
			static u8 cnt[3];	
			if(cnt[0]++>5){cnt[0]=0;	
			RC_Send_Task();	//NRF 发送 TO NRF-G模块
			}
			
			if(cnt[1]++>2){cnt[1]=0;	
			Nrf_Check_Event();//NRF 解算
			PWMIN_TO_RC();//接收机遥感量滤波
			GOL_LINK_TASK();//串口通讯
			}
			delay_ms(5);
			
		}
}
/*********************************************END OF FILE**********************/

