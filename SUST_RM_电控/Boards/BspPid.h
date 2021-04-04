#ifndef PID_H
#define PID_H
#ifndef __PID_H
#define __PID_H



typedef struct{
	double target;		//目标输出量
	double feedback;		//实际输出量
	double Kp;
	double Ki;
	double Kd;
	double e_0;			//当前误差
	double e_1;			//上一次误差
	double sum_e;
}PID;

double PIDCalc(PID *pid);
#endif

#endif
