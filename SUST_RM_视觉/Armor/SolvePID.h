#ifndef Solve_PID
#define Solve_PID

#include "../Base.h"
#include <bits/stdc++.h>

using namespace std;

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0)? -x:x)

static const int16_t MG6020YawAngleMin=6450;
static const int16_t MG6020YawAngleMax=6650;


static const int16_t MG6020PithAngleMin=2980 ;
static const int16_t MG6020PithAngleMax=3850;


struct PID{
    double limit;		//输出限幅
    double target;		//目标输出量
    double feedback;		//实际输出量
    double Kp;
    double Ki;
    double Kd;
    double e_0;			//当前误差
    double e_1;			//上一次误差
    double sum_e;
};

double pid_calc(PID *pid){
    double out;
    double ep, ei, ed;

    pid->e_0 = pid->target - pid->feedback;

    ep = pid->e_0;
    ei = pid->sum_e;
    ed = pid->e_0  - pid->e_1;


    out = pid->Kp*ep + pid->Ki*ei + pid->Kd*ed;

    //out = range(out, -pid->limit, pid->limit);

    cout<<"out  "<<pid->target <<"  "<< pid->feedback<<"  "<<pid->Kp*ep<<endl;

    pid->e_1 = pid->e_0;

    pid->sum_e += pid->e_0;
    return out;
}


#endif
