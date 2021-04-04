#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "BspUsart.h"
#include "BspCan.h"
#include "BspPid.h"
#include "BspLib.h"


typedef struct{
	PID PIDC620Speed[4];
	int C620OutPutSpeed[4];
	int C620InPutSpeed[4];
	RC_Type *RC_Chassis;
	moto_info_t *Motor_Chassis[MOTOR_MAX_NUM];
	int wsad[6];
}ChassisStruct;

void ChassisTask(void const * argument);
#endif

