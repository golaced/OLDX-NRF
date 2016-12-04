
#include "head.h"

#define RX_DR			6		//????
#define TX_DS			5
#define MAX_RT		4

vs16 QH,ZY,XZ;
u8 is_lock=1;
u8 EN_FIX_GPSF=0;
u8 EN_FIX_LOCKWF=0;
u8 EN_CONTROL_IMUF=0;
u8 EN_FIX_INSF=0;
u8 EN_FIX_HIGHF=0;
u8 tx_lock=1;
u8 EN_FIX_GPS=0;
u8 EN_FIX_LOCKW=0;
u8 EN_CONTROL_IMU=0;
u8 EN_FIX_INS=0;
u8 EN_FIX_HIGH=0;
u8 EN_TX_GX=1;
u8 EN_TX_AX=1;
u8 EN_TX_HM=1;
u8 EN_TX_YRP=1;
u8 EN_TX_GPS=1;
u8 EN_TX_HIGH=1;
u8 up_load_set=0;
u8 up_load_pid=0;
u8 key_rc[6]={1,1,1,1,1,1};
u16 Yaw_sb_rc=0;

u8 cnt_rst=0,delta_pitch=0,delta_roll=0,delta_yew=0;



//中值滤波
float GetMedianNum(float * bArray, u16 iFilterLen)
{  
    int i,j;// 循环变量  
    float bTemp;  
      
    // 用冒泡法对数组进行排序  
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 互换  
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 计算中值  
    if ((iFilterLen & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        bTemp = bArray[(iFilterLen + 1) / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  
u16 data_rate;
#define MID_RC_KEY 15
 u8 key_rc_reg[7][MID_RC_KEY];
#define MID_RC_GET 4
float RC_GET[4][MID_RC_GET];
float control_scale=1;
float ypr_sb[3];
u8 loss_nrf=1;
u8 KEY_SEL[4],KEY[8];
RC_GETDATA Rc_Get;
RC_GETDATA Rc_Get_PWM;
void NRF_DataAnl(void)
{ 
int16_t RC_GET_TEMP[4];
float RC_GETR[4][MID_RC_GET];	
u8 temp_key[7];
u8 temp;
	u8 i=0,j,sum = 0;
	for( i=0;i<31;i++)
		sum += NRF24L01_RXDATA[i];
	if(!(sum==NRF24L01_RXDATA[31]))	
	{i=0;
		return;
		}	//??sum}
	if(!(NRF24L01_RXDATA[0]==0x8A))		
		{
			i=0;
		return;
			}	//??sum}
  if(Not_sent)
	for(i=0;i<32;i++)
	NRF24L01_RXDATA_REG[i]=NRF24L01_RXDATA[i];	
			
static u8 ledtx;
if(ledtx)ledtx=0;
else ledtx=1;
LED_STATE(RX_LED,ledtx);		
//--------------
	if(NRF24L01_RXDATA[1]==0x8A)//NRF模块未解码  直接将接受数据发送给了FC模块 由FC模块解码								
	{ 
		data_rate++;
		loss_nrf=0;
		Rc_Get.THROTTLE= (vs16)(NRF24L01_RXDATA[3]<<8)|NRF24L01_RXDATA[4];
		Rc_Get.YAW			= (vs16)(NRF24L01_RXDATA[5]<<8)|NRF24L01_RXDATA[6];
		Rc_Get.PITCH		= (vs16)(NRF24L01_RXDATA[7]<<8)|NRF24L01_RXDATA[8];
		Rc_Get.ROLL 		= (vs16)(NRF24L01_RXDATA[9]<<8)|NRF24L01_RXDATA[10];
    Rc_Get.AUX1			= (vs16)(NRF24L01_RXDATA[11]<<8)|NRF24L01_RXDATA[12];
		Rc_Get.AUX4			= (vs16)(NRF24L01_RXDATA[13]<<8)|NRF24L01_RXDATA[14];
		Rc_Get.AUX3			= (vs16)(NRF24L01_RXDATA[15]<<8)|NRF24L01_RXDATA[16];
		Rc_Get.AUX2			= (vs16)(NRF24L01_RXDATA[17]<<8)|NRF24L01_RXDATA[18];
		Rc_Get.AUX5			= (vs16)(NRF24L01_RXDATA[19]<<8)|NRF24L01_RXDATA[20];

		
		KEY_SEL[0]=(NRF24L01_RXDATA[21])&0x01;
		KEY_SEL[1]=(NRF24L01_RXDATA[21]>>1)&0x01;
		KEY_SEL[2]=(NRF24L01_RXDATA[21]>>2)&0x01;
		KEY_SEL[3]=(NRF24L01_RXDATA[21]>>3)&0x01; 
	  KEY[0]=(NRF24L01_RXDATA[22])&0x01;
		KEY[1]=(NRF24L01_RXDATA[22]>>1)&0x01;
		KEY[2]=(NRF24L01_RXDATA[22]>>2)&0x01;
		KEY[3]=(NRF24L01_RXDATA[22]>>3)&0x01;
		KEY[4]=(NRF24L01_RXDATA[22]>>4)&0x01;
		KEY[5]=(NRF24L01_RXDATA[22]>>5)&0x01;
		KEY[6]=(NRF24L01_RXDATA[22]>>6)&0x01;
		KEY[7]=(NRF24L01_RXDATA[22]>>7)&0x01;
	 }
}

u32 cnt_loss_nrf;
int cnt_timer2_r=0;
u8 cnt_led_rx=0;
void Nrf_Check_Event(void)
{ 
	u8 rx_len =0;
	static u16 cnt_loss_rc=0;
	u8 sta;

	sta= NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);		//??2401?????
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<RX_DR))	//??ing 
	{ 
		cnt_loss_rc=0;
			
				rx_len= NRF_Read_Reg(R_RX_PL_WID);
				
				if(rx_len<33)	//??????33?????,???????
				{
					NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					NRF_DataAnl();	//??2401??????
				
				}
				else 
				{ 
					NRF_Write_Reg(FLUSH_RX,0xff);//?????
				}
				if(cnt_led_rx<2)
				cnt_led_rx++;
				else 
				{
				cnt_led_rx=0;
				}
	}
	else//---------losing_nrf
	 {	
		 if(cnt_loss_rc++>300 )//0.5ms
		 {	
			loss_nrf=1;
			cnt_loss_nrf++;
			NRF_Write_Reg(FLUSH_RX,0xff);//?????		
		 }
  }
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))	//????,?????
	{
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	if(sta & (1<<MAX_RT))//??,????
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}

//-------

PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _MODE mode;
struct _IMU_NAV imu_nav;
int high_sonar,THR;
float Pit,Rol,Yaw;
u8 height_ctrl_mode;

void NRF_Send_ARMED(void)//????
{
	uint8_t i,sum;
	u32 _temp;
	u8 cnt=0;

	NRF24L01_TXDATA[cnt++]=0x88;
	NRF24L01_TXDATA[cnt++]=0x01;
	NRF24L01_TXDATA[cnt++]=0x1C;
	_temp=8400;//BAT_FLY;
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);//添加 gps 姿态 状态通信
	_temp = (int)(31.1234*10000000);
  NRF24L01_TXDATA[cnt++]=BYTE3(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE2(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = 0;
  NRF24L01_TXDATA[cnt++]=BYTE3(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE2(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(fly_controller.imu.roll*100);//(int)(Pitch*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp = (int)(fly_controller.imu.pitch*100);//(int)(Roll*100);// (int)((Target.Yaw+180)*100);//
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
  _temp = (int)(fly_controller.imu.yaw*100);
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp= fly_controller.now.thr;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=data_rate;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp= height_ctrl_mode;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);

	_temp= fly_controller.now.alt_fushion;//ultra_speed;}
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	_temp=0;//baroAlt;//(plane.get.altitude.sonar);//hight
	NRF24L01_TXDATA[cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[cnt++]=BYTE0(_temp);
	//gps mode
	NRF24L01_TXDATA[cnt++]=(0);
	//gps no
	NRF24L01_TXDATA[cnt++]=(1);
	//fly mode
	NRF24L01_TXDATA[cnt++]=(2);
//	NRF24L01_TXDATA[cnt++]=(sys.flow<<3)|(sys.circle<<2)|(sys.avoid<<1)|(sys.gps);
//	NRF24L01_TXDATA[cnt++]=EN_FIX_GPSF;
//	NRF24L01_TXDATA[cnt++]=EN_FIX_LOCKWF;
//	NRF24L01_TXDATA[cnt++]=EN_CONTROL_IMUF;
//	NRF24L01_TXDATA[cnt++]=EN_FIX_INSF;
//	NRF24L01_TXDATA[cnt++]=EN_FIX_HIGHF;
//	//if(ARMED)	NRF24L01_TXDATA[3]=0xA1;
//	//else 			NRF24L01_TXDATA[3]=0xA0;
	sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID1(void)//????
{	vs16 _temp;	u8 i;	u8 sum = 0;

	NRF24L01_TXDATA[0] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[1] = 0x02;	//?????8BSET??
	NRF24L01_TXDATA[2] = 0x1C;
	NRF24L01_TXDATA[3] = 0xAD;
	
	_temp =SPID.OP;
	NRF24L01_TXDATA[4]=BYTE1(_temp);
	NRF24L01_TXDATA[5]=BYTE0(_temp);
	_temp =SPID.OI;
	NRF24L01_TXDATA[6]=BYTE1(_temp);
	NRF24L01_TXDATA[7]=BYTE0(_temp);
	_temp = SPID.OD;
	NRF24L01_TXDATA[8]=BYTE1(_temp);
	NRF24L01_TXDATA[9]=BYTE0(_temp);
	_temp =SPID.IP;
	NRF24L01_TXDATA[10]=BYTE1(_temp);
	NRF24L01_TXDATA[11]=BYTE0(_temp);
	_temp =SPID.II;
	NRF24L01_TXDATA[12]=BYTE1(_temp);
	NRF24L01_TXDATA[13]=BYTE0(_temp);
	_temp = SPID.ID;
	NRF24L01_TXDATA[14]=BYTE1(_temp);
	NRF24L01_TXDATA[15]=BYTE0(_temp);
	_temp = SPID.YP;//.YP * 1;
	NRF24L01_TXDATA[16]=BYTE1(_temp);
	NRF24L01_TXDATA[17]=BYTE0(_temp);
	_temp = SPID.YI;//SPID.YI * 1;
	NRF24L01_TXDATA[18]=BYTE1(_temp);
	NRF24L01_TXDATA[19]=BYTE0(_temp);
  _temp = SPID.YD;//.YD * 1;
	NRF24L01_TXDATA[20]=BYTE1(_temp);
	NRF24L01_TXDATA[21]=BYTE0(_temp);
	
	

sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_PID2(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x03;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;
	_temp = 	  HPID.OP;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OI;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	HPID.OD;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	0*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
		_temp = 	0*100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	
sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void NRF_Send_RC_Sensor(void)//????
{	vs16 _temp;	u8 _cnt=0;u8 i;u8 sum = 0;

	NRF24L01_TXDATA[_cnt++] = 0x88;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x04;	//?????8BSET??
	NRF24L01_TXDATA[_cnt++] = 0x1C;
	NRF24L01_TXDATA[_cnt++]=	0xAD;

	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =  0;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	
	sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}


void NRF_Send_RC_GPS(void)
{u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	NRF24L01_TXDATA[_cnt++]=0x88;
	NRF24L01_TXDATA[_cnt++]=0x05;
	NRF24L01_TXDATA[_cnt++]=0x1C;//功能字
	NRF24L01_TXDATA[_cnt++]=0xAD;
	_temp32 =  imu_nav.gps.J;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp32);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp32);


sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);;
}


void NRF_Send_RC_DEBUG1(void)
{u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	NRF24L01_TXDATA[_cnt++]=0x88;
	NRF24L01_TXDATA[_cnt++]=0x08;
	NRF24L01_TXDATA[_cnt++]=0x1C;
	for(i=0;i<14;i++){
	_temp =  BLE_DEBUG[i];	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
  }
sum = 0;
	for(i=0;i<31;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[31]=sum;
	
	NRF_TxPacket(NRF24L01_TXDATA,32);
}

void RC_Send_Task(void)//--------------NRF发送
{
static u16 cnt[4]={0,0,0,0};
static u8 state;
if(KEY[3])
	NRF_Send_RC_DEBUG1();
else{
switch(state)
{
		case 0:NRF_Send_ARMED();state=1;
		break;
		case 1:
		NRF_Send_RC_GPS();state=2;
		break;
		case 2: NRF_Send_RC_DEBUG1();state=3;
		break;
		case 3:
		if(cnt[0]==0){cnt[0]=1;
		NRF_Send_RC_PID1();}
		else
		{cnt[0]=0;
		NRF_Send_RC_PID2();}
		state=0;
		break;
}
}
static u8 ledtx;
if(ledtx)ledtx=0;
else ledtx=1;
LED_STATE(TX_LED,ledtx);
}