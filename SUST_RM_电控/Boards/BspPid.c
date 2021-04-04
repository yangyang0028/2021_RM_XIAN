#include "BspPid.h"

double PIDCalc(PID *pid){
	double out;
	double ep, ei, ed;
	pid->sum_e += pid->e_0;

	pid->e_0 = pid->target - pid->feedback;
	ep = pid->e_0;
	ei = pid->sum_e;
	ed = pid->e_0  - pid->e_1;

	out = pid->Kp*ep + pid->Ki*ei + pid->Kd*ed;
	
	pid->e_1 = pid->e_0;
	return out;
}
