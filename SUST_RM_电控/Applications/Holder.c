#include "Holder.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "usart.h"
#include "main.h"
#include "Shoot.h"
#include "BMI088driver.h"


fp32 gyro[3], accel[3], temp;
HolderStruct Holder;

uint8_t UART1_temp[10];
int16_t ansX,ansY;
int numVI ;

extern RC_Type remote_control;
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern ShootStruct  Shoot;


const int16_t MG6020YawBegin=2450;
const int16_t MG6020PithBegin=3400;  //3850

static const int16_t MG6020YawAngleMin=2250;
static const int16_t MG6020YawAngleMax=2650;


static const int16_t MG6020PithAngleMin=2700 ;
static const int16_t MG6020PithAngleMax=4100;
	


void HolderOutPutConstrain(int *ans){
	
	static const int16_t MG6020CurrentMin=-28000;
	static const int16_t MG6020CurrentMax=28000;
	
	if(*ans>MG6020CurrentMax){
		*ans=MG6020CurrentMax;
	}
	if(*ans<MG6020CurrentMin){
		*ans=MG6020CurrentMin;
	}
}

void HolderInPutConstrain(HolderStruct *Holder){
	

	if(Holder->MG6020InPutAngle[0]<MG6020PithAngleMin) Holder->MG6020InPutAngle[0]=MG6020PithAngleMin;
	if(Holder->MG6020InPutAngle[0]>MG6020PithAngleMax) Holder->MG6020InPutAngle[0]=MG6020PithAngleMax;
	if(Holder->MG6020InPutAngle[1]<MG6020YawAngleMin) Holder->MG6020InPutAngle[1]=MG6020YawAngleMin;
	if(Holder->MG6020InPutAngle[1]>MG6020YawAngleMax) Holder->MG6020InPutAngle[1]=MG6020YawAngleMax;
}

void HolderInit(HolderStruct *Holder){
	Holder->RC_Holder=&remote_control;
	Holder->MG6020InPutAngle[0]=MG6020PithBegin;
	Holder->MG6020InPutAngle[1]=MG6020YawBegin;
	
	for(int i=0;i<2;i++){
		Holder->MG6020OutPutCurrent[i]=0;
		Holder->Motor_Holder[i]=&motor_info[i+5];
	}
	Holder->MG6020AnglePID[0]=(PID){0,0, 4,0,0, 0,0,0};
	Holder->MG6020SpeedPID[0]=(PID){0,0, 35,0,0, 0,0,0};

	Holder->MG6020AnglePID[1]=(PID){0,0, 3,0,0, 0,0,0};
	Holder->MG6020SpeedPID[1]=(PID){0,0, 40,0,0, 0,0,0};
	
	
}



static double max(double a,double b){
	if(a>b) return a;
	else return b;
}
static double min(double a,double b){
	if(a<b) return a;
	else return b;
}

void HolderSetMode(HolderStruct *Holder){
	if(Holder->RC_Holder->press_r && Holder->numVI < numVI){
		Holder->numVI = numVI;
		Holder->MG6020InPutAngle[0]-=ansY/5;
		Holder->MG6020InPutAngle[1]+=ansX/5;
	}else{
		Holder->MG6020InPutAngle[0]+=min(max(Holder->RC_Holder->y/3,-0.25),0.25);
		Holder->MG6020InPutAngle[1]-=min(max(Holder->RC_Holder->x/3,-0.25),0.25);
	}
}

void HolderPid(HolderStruct *Holder){
	HolderInPutConstrain(Holder);
	for(int i=0;i<2;i++){
		Holder->MG6020AnglePID[i].target=Holder->MG6020InPutAngle[i];
		Holder->MG6020AnglePID[i].feedback=Holder->Motor_Holder[i]->rotor_angle;		
		
		Holder->MG6020SpeedPID[i].target=PIDCalc(&Holder->MG6020AnglePID[i]);
		Holder->MG6020SpeedPID[i].feedback=Holder->Motor_Holder[i]->rotor_speed;
		
		Holder->MG6020OutPutCurrent[i]=PIDCalc(&Holder->MG6020SpeedPID[i]);
		HolderOutPutConstrain(&Holder->MG6020OutPutCurrent[i]);
	}
	set_motor_voltage_MG6020(Holder->MG6020OutPutCurrent[0],Holder->MG6020OutPutCurrent[1],Shoot.ShootOutIntegral,0);
}


void HolderTask(void const * argument){
	vTaskDelay(100);
	HolderInit(&Holder);
  while(1){
		HolderSetMode(&Holder);
		HolderPid(&Holder);
		osDelay(1);
  }
}
