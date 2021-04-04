#include "Shoot.h"
#include "cmsis_os.h"
#include "tim.h"
#include "BspCan.h"

ShootStruct  Shoot;
extern RC_Type remote_control;
extern moto_info_t motor_info[MOTOR_MAX_NUM];

 int Timing;
 int PWMTiming;
	
void ShootInit (ShootStruct *Shoot){
	Shoot->HolderLeftMode=&remote_control.press_l;
	Shoot->HolderRightMode=&remote_control.switch_right;	
	Shoot->FeedBackAngle=&motor_info[7].rotor_angle;
	Shoot->FeedBackCurrent=&motor_info[7].torque_current;
	Shoot->FeedBackSpeed=&motor_info[7].rotor_speed;
	
	Shoot->ShootAngle=(PID){0,0, 1,0,0, 0,0,0};
	Shoot->ShootSpeed=(PID){0,0,600,0,0, 0,0,0};
	Shoot->ShootCurrent=(PID){0,0, 0.0,0,0, 0,0,0};
	
	
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	Shoot->ShootSign=0;
	
	Shoot->RC_Shoot=&remote_control;
}
void ShootOutPutConstrain(int *ans){
	
	static const int16_t ShootCurrentMin=0;
	static const int16_t ShootCurrentMax=9500;
	
	if(*ans>ShootCurrentMax){
		*ans=ShootCurrentMax;
	}
	if(*ans<ShootCurrentMin){
		*ans=ShootCurrentMin;
	}
}
void ShootPid(ShootStruct *Shoot){
//	if(Shoot->RC_Shoot->v&KEY_PRESSED_OFFSET_SHIFT){
//		Shoot->ShootOutIntegral=-1000;
//		return ;
//	}
	Shoot->ShootSpeed.target=Shoot->ShootInIntegral;
	Shoot->ShootSpeed.feedback=Shoot->SpeedIntegral;
	
	Shoot->ShootOutIntegral=PIDCalc(&Shoot->ShootSpeed);
	ShootOutPutConstrain(&Shoot->ShootOutIntegral);
}
void ShootMain(ShootStruct *Shoot){
		PWMTiming++;
		if(PWMTiming>7200) PWMTiming=7200;
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWMTiming/5); 
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, PWMTiming/5); 	
	if(*Shoot->HolderLeftMode){
		if(Timing==1 && Shoot->ShootSign==0){
			Shoot->ShootSign=1;
			Shoot->ShootInIntegral=Shoot->SpeedIntegral+120;
		}
		Timing++;
		if(Timing>5000) Timing=0;
	}else {
				Shoot->ShootSpeed.sum_e=0;
		Timing=0;
		Shoot->ShootSign=0;
		Shoot->ShootOutIntegral=0;
	}
}
void gatSpeedIntegral(ShootStruct *Shoot){
	Shoot->SpeedIntegral+=*Shoot->FeedBackSpeed/100;
}
void ShootTask(void const * argument){
	vTaskDelay(100);
	ShootInit(&Shoot);
  while(1){ 
		gatSpeedIntegral(&Shoot);
		ShootPid(&Shoot);
		ShootMain(&Shoot);
		osDelay(1);
  }
}

