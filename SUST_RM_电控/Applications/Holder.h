#ifndef __HOLDER_H
#define __HOLDER_H
#include "BspUsart.h"
#include "BspCan.h"
#include "BspPid.h"


typedef struct{
	PID MG6020AnglePID[2];
	PID MG6020SpeedPID[2];
	
	int MG6020OutPutCurrent[2];
	double MG6020InPutAngle[2];
	
	RC_Type *RC_Holder;
	
	moto_info_t *Motor_Holder[MOTOR_MAX_NUM];
	
	int numVI;
	
}HolderStruct;

void HolderTask(void const * argument);
#endif

