#include "stm32f10x.h"
#include "UARTs.h"
#include "PWM.h"
#include "string.h"
#include "math.h"
#include "motor.h"
#include "24l01.h" 
#include "JY901.h"
#include "delay.h"
#include "timer.h"


struct SAcc 		stcAcc;
struct SGyro		stcGyro;
struct SAngle 	stcAngle;
int tickFlag;
u8 tmp_buf[33];		 
char buf[33];
float channel[4];
int key[2];

int JZ_Angle[3];
int JZ=0;
int JZ_IMU=0;

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);stcAngle.Angle[0]=stcAngle.Angle[0]+32768;break;
		}
		ucRxCnt=0;
	}
}



int main(void)
{
	TIM3_PWM_Init(1000-1,72);
	TIM4_IT_Init(10000-1,72);
	Initial_UART1(115200);
	NRF24L01_Init();

	tickFlag=0;
	while(NRF24L01_Check())
	{	}
	NRF24L01_RX_Mode();
	
  while(1)
	{
		if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,解析出来
		{
			tmp_buf[32]=0;//加入字符串结束符
			for(int i=0;i<33;i++)
				buf[i]=tmp_buf[i];
			sscanf(buf,"*C%fC%fC%fC%fC%dC%d",&channel[0],&channel[1],&channel[2],&channel[3],&key[0],&key[1]);
		}
		else
		{
			while(NRF24L01_Check()){}
		}
		if(JZ_IMU==0)
		{
			for(int i=0;i<3;i++)
				JZ_Angle[i]=stcAngle.Angle[i];
			JZ_IMU=1;
		}
		if(tickFlag>=1)
		{
			for(int i=0;i<3;i++)
				stcAngle.Angle[i]=stcAngle.Angle[i]-JZ_Angle[i];
			gyroCal(channel,stcGyro);
			angleCal(channel,stcAcc,stcAngle);
			motorRun(channel,key);
			tickFlag=0;
		}
	}
}

void TIM4_IRQHandler(void) //TIM4 中断函数
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET)
	{
		tickFlag++;
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}  
}  
