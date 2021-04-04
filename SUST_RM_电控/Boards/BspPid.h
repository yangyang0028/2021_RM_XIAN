#ifndef PID_H
#define PID_H
#ifndef __PID_H
#define __PID_H



typedef struct{
	double target;		//Ŀ�������
	double feedback;		//ʵ�������
	double Kp;
	double Ki;
	double Kd;
	double e_0;			//��ǰ���
	double e_1;			//��һ�����
	double sum_e;
}PID;

double PIDCalc(PID *pid);
#endif

#endif
