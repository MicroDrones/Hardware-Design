#include "motor.h"
#include "math.h"
#include "JY901.h"

struct PID  gyro_pit={1.2,0.02,0.6,0,0,0,0};
struct PID  gyro_rol={1.2,0.02,0.6,0,0,0,0};
struct PID  gyro_yaw={3.8,0.55,0.1,0,0,0,0};

struct PID angle_pit={1.3,0,0.2,0,0,0,0};
struct PID angle_rol={1.3,0,0.2,0,0,0,0};

//���ٶȱջ�����������
float gyro_pit_feed;
float gyro_rol_feed;
float gyro_yaw_feed; 
//�Ƕȱջ�����������
float angle_pit_feed;
float angle_rol_feed;
float angle_yaw_feed;
//���ſ�����
float thr_feed;

float THRUST=0;

float PWM[4];
int motorPWM[4];

float feedbackCal(float target,float current,struct PID pid)
{
	pid.error=target-current;//������

	pid.IntergralError+=pid.error;//������
	pid.IntergralError*=0.95;//�����˲�
	//�����޷�
	if(pid.IntergralError>=100)pid.IntergralError=100;
	if(pid.IntergralError<=-100)pid.IntergralError=-100;
	//��������޶�
	if(pid.error*pid.IntergralError>0)
		pid.IntergralError=0;
	pid.DiffError=pid.error-pid.LastError;//���΢��
	pid.LastError=pid.error;//������
	//PID����,�޷�
	pid.output=pid.P*pid.error+pid.I*pid.IntergralError-pid.D*pid.DiffError;
	if(pid.output>300)pid.output=300;
	return pid.output;
}
void angleCal(float CH[],struct SAcc acc,struct SAngle angle)
{
	//�Ƕȱջ���������
	angle_pit_feed=feedbackCal(-0.02*CH[0],180.0*angle.Angle[0]/32768,angle_pit);
	angle_rol_feed=feedbackCal(0.02*CH[1],180.0*angle.Angle[1]/32768,angle_rol);
	//���Ŷ�������
	THRUST=THRUST+0.001*CH[2];
	if(THRUST<=0)THRUST=0;
	if(THRUST>=900)THRUST=900;
	thr_feed=THRUST;
}
void gyroCal(float CH[],struct SGyro gyro)
{
	//���ٶȱջ���������
	gyro_pit_feed=feedbackCal(angle_pit_feed, 2000.0*gyro.w[0]/32768,gyro_pit);
	gyro_rol_feed=feedbackCal(angle_rol_feed,-2000.0*gyro.w[1]/32768,gyro_rol);
	gyro_yaw_feed=feedbackCal(0.05*CH[3],2000.0*gyro.w[2]/32768,gyro_yaw);
}
void motorRun(float CH[],int key[])
{
	//�����������Ե���
	PWM[0]=thr_feed+gyro_rol_feed+0.05*CH[1]+gyro_pit_feed-0.05*CH[0]+gyro_yaw_feed;
	PWM[1]=thr_feed+gyro_rol_feed+0.05*CH[1]-gyro_pit_feed+0.05*CH[0]-gyro_yaw_feed;
	PWM[2]=thr_feed-gyro_rol_feed-0.05*CH[1]+gyro_pit_feed-0.05*CH[0]-gyro_yaw_feed;
	PWM[3]=thr_feed-gyro_rol_feed-0.05*CH[1]-gyro_pit_feed+0.05*CH[0]+gyro_yaw_feed;

	for(int i=0;i<4;i++)
	{
		if(PWM[i]<=0||THRUST<=10)PWM[i]=0;
	}

	if(key[1]==0)
	{
		THRUST=0;
		for(int i=0;i<4;i++)PWM[i]=0;
	}
	for(int i=0;i<4;i++)motorPWM[i]=PWM[i];
	TIM3->CCR1=PWM[0];
	TIM3->CCR2=PWM[1];
	TIM3->CCR3=PWM[2];
	TIM3->CCR4=PWM[3];
	
}
