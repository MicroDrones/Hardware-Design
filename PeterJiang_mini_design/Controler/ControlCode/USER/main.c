#include "stm32f10x.h"
#include "string.h"
#include "math.h"
#include "24l01.h" 
#include "adc.h"
#include "key.h"
#include "delay.h"
#include "UARTs.h"

volatile u8 tickFlag;
u8 tmp_buf[33];		    
int channel[6];
int J_CH[4];
int JZ;

void SysTick_Handler(void)
{
	tickFlag=1;
	
}
int map(int org,int oldDown,int oldUp,int newDown,int newUp)
{
	float temp=(org-oldDown)*(newUp-newDown)/(oldUp-oldDown);
	return (int)temp+newDown;
}
int main(void)
{
	SysTick_Config(SystemCoreClock/50000);//set sysTIck_IT as 100ms
	NRF24L01_Init(); 
	Adc_Init();
	Key_Init();
	delay_init();
	Initial_UART1(115200);
	//tick=0;
	tickFlag=0;
	JZ=5;
	while(NRF24L01_Check())
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	}
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	NRF24L01_TX_Mode();
	
	//int temp=0;
  while(1)	
	{
		channel[2]=map(Get_Adc_Average(0,2),0,4096,-999,999);
		channel[3]=map(Get_Adc_Average(1,2),0,4096,-999,999);
		channel[0]=map(Get_Adc_Average(2,2),0,4096,-999,999);
		channel[1]=map(Get_Adc_Average(3,2),0,4096,-999,999);
		if(JZ>0)
		{
			for(int i=0;i<4;i++)
				J_CH[i]+=channel[i];
			JZ--;
			if(JZ==0)
			{
				for(int i=0;i<4;i++)
					J_CH[i]=J_CH[i]/5;
			}
		}
		for(int i=0;i<4;i++)
		{
			channel[i]=channel[i]-J_CH[i];
			if(channel[i]<0)
				channel[i]=-channel[i]*channel[i]/1000;
			else
				channel[i]=channel[i]*channel[i]/1000;
		}

		channel[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8);
		channel[5]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
		if(tickFlag==1)
		{
			sprintf(tmp_buf,"*C%dC%dC%dC%dC%dC%d",-channel[0],channel[1],-channel[2],channel[3],channel[4],channel[5]);
			UART1_Put_String(tmp_buf);
			UART1_Put_String("\n");
			
			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
				GPIO_SetBits(GPIOA,GPIO_Pin_7);
			else 
				GPIO_ResetBits(GPIOA,GPIO_Pin_7);
			tickFlag=0;
		}
	}
}
