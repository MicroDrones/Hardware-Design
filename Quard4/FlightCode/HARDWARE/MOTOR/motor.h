#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"
#include "JY901.h"

struct PID
{
	float P;
	float I;
	float D;
	float error;
	float LastError;
	float IntergralError;
	float DiffError;
	float output;
};
void Jzero(float CH[]);
float feedbackCal(float target,float input,struct PID pid);
void gyroCal(float CH[],struct SGyro gyro);
void angleCal(float CH[],struct SAcc acc,struct SAngle angle);
void motorRun(float CH[],int key[]);

#endif
