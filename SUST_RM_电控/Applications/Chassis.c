#include "Chassis.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

ChassisStruct Chassis;

extern RC_Type remote_control;
extern moto_info_t motor_info[MOTOR_MAX_NUM];

void ChassisOutPutSpeed(int *ans){
	
	static int16_t C620OutPutCurrentMin=-4000;
	static int16_t C620OutPutCurrentMax=4000;
	
	if(*ans>C620OutPutCurrentMax){
		*ans=C620OutPutCurrentMax;
	}
	if(*ans<C620OutPutCurrentMin){
		*ans=C620OutPutCurrentMin;
	}
}

void ChassisInPutSpeed(int *ans){
	
	static int16_t C620InPutCurrentMin=0;
	static int16_t C620InPutCurrentMax=4000;
	
	if(*ans>C620InPutCurrentMax){
		*ans=C620InPutCurrentMax;
	}
	if(*ans<C620InPutCurrentMin){
		*ans=C620InPutCurrentMin;
	}
}

void ChassisInit(ChassisStruct *Chassis){
	Chassis->RC_Chassis=&remote_control;
	for(int i=0;i<4;i++){
		Chassis->Motor_Chassis[i]=&motor_info[i+1];
		Chassis->PIDC620Speed[i]=(PID){0,0, 12,0,0, 0,0,0};
		Chassis->C620OutPutSpeed[i]=0;
		Chassis->C620InPutSpeed[i]=0;
	}
}

static int max(int a,int b){
	if(a>b) return a;
	else return b;
}
static int min(int a,int b){
	if(a<b) return a;
	else return b;
}


void ChassisModeSetRemote(ChassisStruct *Chassis){
	if(Chassis->RC_Chassis->v&KEY_PRESSED_OFFSET_W){
		Chassis->wsad[0]+=5;
	}else{
		if(Chassis->wsad[0]>500) Chassis->wsad[0]-=500;
		else Chassis->wsad[0]=0;
	}
	if(Chassis->RC_Chassis->v&KEY_PRESSED_OFFSET_S){
		Chassis->wsad[1]+=5;
	}else{
		if(Chassis->wsad[1]>500) Chassis->wsad[1]-=500;
		else Chassis->wsad[1]=0;
	}
	if(Chassis->RC_Chassis->x >= 10 || Chassis->RC_Chassis->x <= -10 ){
		Chassis->wsad[2]+=min(max(Chassis->RC_Chassis->x/3,-5),5);
	}else{
		if(Chassis->wsad[2]>500) Chassis->wsad[2]-=500;
		else if(Chassis->wsad[2]<-500) Chassis->wsad[2]+=500;
		else Chassis->wsad[2]=0;
	}
	if(Chassis->RC_Chassis->x >= 10 || Chassis->RC_Chassis->x <= -10 ){
		Chassis->wsad[3]-=min(max(Chassis->RC_Chassis->x/3,-5),5);
	}else{
		if(Chassis->wsad[3]>500) Chassis->wsad[3]-=500;
		else if(Chassis->wsad[3]<-500) Chassis->wsad[3]+=500;
		else Chassis->wsad[3]=0;
	}
	
	if(Chassis->RC_Chassis->v&KEY_PRESSED_OFFSET_A){
		Chassis->wsad[4]+=5;
	}else{
		if(Chassis->wsad[4]>500) Chassis->wsad[4]-=500;
		else Chassis->wsad[4]=0;
	}
	
	if(Chassis->RC_Chassis->v&KEY_PRESSED_OFFSET_D){
		Chassis->wsad[5]+=5;
	}else{
		if(Chassis->wsad[5]>500) Chassis->wsad[5]-=500;
		else Chassis->wsad[5]=0;
		
	}
	Chassis->C620InPutSpeed[0]=-Chassis->wsad[4]+Chassis->wsad[5]+Chassis->wsad[2]-Chassis->wsad[3];
	Chassis->C620InPutSpeed[1]=-Chassis->wsad[1]+Chassis->wsad[0]+Chassis->wsad[2]-Chassis->wsad[3];
	Chassis->C620InPutSpeed[2]=+Chassis->wsad[4]-Chassis->wsad[5]+Chassis->wsad[2]-Chassis->wsad[3];
	Chassis->C620InPutSpeed[3]=+Chassis->wsad[1]-Chassis->wsad[0]+Chassis->wsad[2]-Chassis->wsad[3];	
}

void ChassisPowerlimit(ChassisStruct *Chassis){
	int num=0;
	double proportion=0;
	for(int i=0;i<4;i++){
		num+=Chassis->Motor_Chassis[i]->set_voltage;
	}
	num=num*1000/16384;
	if(num<10000) return ;
	proportion=1-((num-10000)/num);
	for(int i=0;i<4;i++){
		Chassis->C620OutPutSpeed[i]=Chassis->C620OutPutSpeed[i]*proportion;
	}
}


void ChassisPid(ChassisStruct *Chassis){
	for(int i=0;i<4;i++){
		Chassis->PIDC620Speed[i].target=Chassis->C620InPutSpeed[i];
		Chassis->PIDC620Speed[i].feedback = Chassis -> Motor_Chassis[i]->rotor_speed;
		Chassis->C620OutPutSpeed[i]=PIDCalc(&Chassis->PIDC620Speed[i]);
		ChassisOutPutSpeed(&Chassis->C620OutPutSpeed[i]);
	}
	ChassisPowerlimit(Chassis);
	set_motor_voltage_C620(	Chassis->C620OutPutSpeed[0],
													Chassis->C620OutPutSpeed[1],
													Chassis->C620OutPutSpeed[2],
													Chassis->C620OutPutSpeed[3]);
}
void ChassisTask(void const * argument){
	vTaskDelay(100);
	ChassisInit(&Chassis);
  while(1){
		ChassisModeSetRemote(&Chassis);
		ChassisPid(&Chassis);
		osDelay(1);
  }
}
