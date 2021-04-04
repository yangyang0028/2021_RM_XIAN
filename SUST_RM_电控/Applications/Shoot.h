#ifndef __SHOOT_H
#define __SHOOT_H
#include "BspUsart.h"
#include "BspCan.h"
#include "BspPid.h"

void ShootTask(void const *argument);


typedef struct{
	int *HolderLeftMode;
	int *HolderRightMode;
	PID ShootAngle;
	PID ShootSpeed;
	PID ShootCurrent;
	
	uint16_t *FeedBackAngle;
	int16_t *FeedBackSpeed;
	int16_t *FeedBackCurrent;	
	
	RC_Type *RC_Shoot;
	
	double SpeedIntegral;
	
	int PWMOutPut;
	
	int ShootOutIntegral;
	int ShootInIntegral;
	int ShootSign; 
}ShootStruct;
#endif

