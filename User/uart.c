#include "head.h"


void usart1_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
		//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 256000;                 /*设置波特率为115200*/
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
	USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
	/*完成串口COM1的时钟配置、GPIO配置，根据上述参数初始化并使能*/

	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
uint8_t UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART3, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART3->SR & USART_FLAG_TXE));
	return DataToSend;
}

void UsartSend_GOL_LINK(uint8_t ch)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}



void Send_RC(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x66;//功能字
	data_to_send[_cnt++]=0;//数据量
	for(i=0;i<RX_PLOAD_WIDTH;i++){
	_temp = NRF24L01_RXDATA_REG[i];
	data_to_send[_cnt++]=BYTE0(_temp);}
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_RC_PWM(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x88;//功能字
	data_to_send[_cnt++]=0;//数据量
  
	_temp=Rc_Get_PWM.PITCH;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.ROLL;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.THROTTLE;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.YAW;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=Rc_Get_PWM.AUX1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.RST;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.HEIGHT_MODE;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_PWM.POS_MODE;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}
u8 Not_sent=0;
void GOL_LINK_TASK(void)
{
static u8 cnt[4];
Not_sent=0;	
Send_RC();
Send_RC_PWM();	
Not_sent=1;
if(cnt[1]++>1)
{cnt[1]=0;
	
	
}
}

//---


void data_check_float(float *data,float *data_r,float in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}

void data_check_int(int *data,int* data_r,int in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}
int16_t BLE_DEBUG[16];
struct _FLY fly_controller,fly_controller_r;
 void Data_Receive_Anl(u8 *data_buf,u8 num)//fly_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i,j;

	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{
		j++;return;	
	}		//判断帧头
	if(*(data_buf+2)==0x05)								//判断功能字0x80 飞控IMU数据
	{
	     
			data_check_float(&fly_controller.imu.roll,&fly_controller_r.imu.roll,(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-180,180);
			data_check_float(&fly_controller.imu.pitch , &fly_controller_r.imu.pitch,(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-180,180);
			data_check_float(&fly_controller.imu.yaw, &fly_controller_r.imu.yaw,(float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10,-180,180);
			data_check_int(&fly_controller.imu.gro_x, &fly_controller_r.imu.gro_x,((vs16)(*(data_buf+10)<<8)|*(data_buf+11)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_y, &fly_controller_r.imu.gro_y,((vs16)(*(data_buf+12)<<8)|*(data_buf+13)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_z, &fly_controller_r.imu.gro_z,((vs16)(*(data_buf+14)<<8)|*(data_buf+15)),-1024,1024);
			
			data_check_int(&fly_controller.imu.acc_x, &fly_controller_r.imu.acc_x,((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_y, &fly_controller_r.imu.acc_y,((vs16)(*(data_buf+18)<<8)|*(data_buf+19)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_z, &fly_controller_r.imu.acc_z,((vs16)(*(data_buf+20)<<8)|*(data_buf+21)),-6000,6000);
			
			data_check_int(&fly_controller.imu.q0,		&fly_controller_r.imu.q0,((vs16)(*(data_buf+22)<<8)|*(data_buf+23)),-2000,2000);
			data_check_int(&fly_controller.imu.q1, 		&fly_controller_r.imu.q1,((vs16)(*(data_buf+24)<<8)|*(data_buf+25)),-2000,2000);
			data_check_int(&fly_controller.imu.q2, 		&fly_controller_r.imu.q2,((vs16)(*(data_buf+26)<<8)|*(data_buf+27)),-2000,2000);
			data_check_int(&fly_controller.imu.q3, 		&fly_controller_r.imu.q3,((vs16)(*(data_buf+28)<<8)|*(data_buf+29)),-2000,2000);
		 //  en_save = *(data_buf+30);//sd_使能存储		

	}
	else if(*(data_buf+2)==0x01)								//判断功能字0x82 飞控姿态PID控制量
	{
	
			fly_controller.set.pitch = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.set.roll = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.set.yaw=(float) ((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
	
	}
	else if(*(data_buf+2)==0x02)								//飞控光流PID控制
	{  fly_controller.set.pos[0]= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		 fly_controller.set.pos[1]= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		 fly_controller.now.pos[0]= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		 fly_controller.now.pos[1]= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		 fly_controller.set.spd[0]= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		 fly_controller.set.spd[1]= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		 fly_controller.now.spd[0]= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		 fly_controller.now.spd[1]= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		 fly_controller.now.nav[0]= (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		 fly_controller.now.nav[1]= (float)((vs16)(*(data_buf+22)<<8)|*(data_buf+23))/10;
	}
		else if(*(data_buf+2)==0x03)								//飞控高度PID控制
	{
	
			data_check_int(&fly_controller.set.alt ,&fly_controller_r.set.alt , ((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt ,&fly_controller_r.now.alt , ((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-10*100,10*100);
		  data_check_float(&fly_controller.set.spd_alt,&fly_controller_r.set.spd_alt , ((vs16)(*(data_buf+8)<<8)|*(data_buf+9)),-500,500);
	    fly_controller.now.spd_alt = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.now.thr = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		  data_check_int(&fly_controller.now.alt_bmp ,&fly_controller_r.now.alt_bmp , ((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt_fushion ,&fly_controller_r.now.alt_fushion , ((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/10,-10*100,10*100);
		
	}
			else if(*(data_buf+2)==0x04)								//飞控模式
	{
	
			//en_save = *(data_buf+4);//sd_使能存储		
	}
			else if(*(data_buf+2)==0x06)								//飞控使用光流数据
	{
	
			 fly_controller.flow.spd_f[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.flow.spd_f[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.flow.spd[0] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.flow.spd[1] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.flow.pos_t[0] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	     fly_controller.flow.pos_t[1] = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		   fly_controller.flow.pos[0] = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	     fly_controller.flow.pos[1] = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
	
		
	}
				else if(*(data_buf+2)==0x07)								//飞控使用光流数据
	{
			SPID.OP= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			SPID.OI= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			SPID.OD= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			SPID.IP= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			SPID.II= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			SPID.ID= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			SPID.YP= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			SPID.YI= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			SPID.YD= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
			HPID.OP= ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
			HPID.OI= ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
			HPID.OD= ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		
	}
				else if(*(data_buf+2)==0x08)								//飞控使用GPS数据
	{
	BLE_DEBUG[0]=((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	BLE_DEBUG[1]=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
	BLE_DEBUG[2]=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	BLE_DEBUG[3]=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
	BLE_DEBUG[4]=((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	BLE_DEBUG[5]=((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
	BLE_DEBUG[6]=((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	BLE_DEBUG[7]=((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
	BLE_DEBUG[8]=((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
	BLE_DEBUG[9]=((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
	BLE_DEBUG[10]=((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
	BLE_DEBUG[11]=((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	BLE_DEBUG[12]=((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
	BLE_DEBUG[13]=((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
	BLE_DEBUG[14]=((vs16)(*(data_buf+32)<<8)|*(data_buf+33));
	BLE_DEBUG[15]=((vs16)(*(data_buf+34)<<8)|*(data_buf+35));		
		
	/*
	fly_controller.gps.J = 			  ((long)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));//W
	fly_controller.gps.W = 			  ((long)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));//J
	fly_controller.gps.gps_mode =  *(data_buf+12);//W
	fly_controller.gps.star_num =  *(data_buf+13);//J
	fly_controller.gps.X_O = 			((long)(*(data_buf+14)<<24)|(*(data_buf+15)<<16)|(*(data_buf+16)<<8)|*(data_buf+17));//W
	fly_controller.gps.Y_O = 			((long)(*(data_buf+18)<<24)|(*(data_buf+19)<<16)|(*(data_buf+20)<<8)|*(data_buf+21));//J
	fly_controller.gps.X_UKF = 		((long)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25));//W
	fly_controller.gps.Y_UKF = 		((long)(*(data_buf+26)<<24)|(*(data_buf+27)<<16)|(*(data_buf+28)<<8)|*(data_buf+29));//J
		*/
	}
	//---------------------------------------------------------------------
	else if(*(data_buf+2)==0x71)								//判断功能字0x81 飞控Sensor数据
	{
			fly_controller.sensor.accx = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.sensor.accy = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.sensor.accz= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.sensor.grox = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.sensor.groy = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.sensor.groz = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			fly_controller.sensor.hmx = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			fly_controller.sensor.hmy = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			fly_controller.sensor.hmz = ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		  fly_controller.sensor.bmp = ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		  fly_controller.sensor.temp = ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		  fly_controller.sensor.sonar = ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
}


u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART3_IRQHandler(void)
{ 
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	
		//ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}

}
