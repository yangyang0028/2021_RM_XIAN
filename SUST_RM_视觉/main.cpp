#include "Armor/Armor.h"
#include "Buff/Buff.h"
#include "Base.h"
#include "Armor/SolvePID.h"
#include "SerialPort/SerialPort.h"
Mat image;
char buf[200];
static volatile unsigned int ProduceIndex=0;     // 图像生成序号，用于线程之间逻辑
static volatile unsigned int ConsumptionIndex=0; // 图像消耗序号
int16_t ansX,ansY;

static PID YawPID={0,0,0, 1,0,0, 0,0,0};

static PID PithPID={0,0,0, 1,0,0, 0,0,0};

static const int16_t targetYaw=283;

static const int16_t targetPith=211;


void CameraSolve(){
    VideoCapture capture;
    #if VIDEO_READ_IO
        #if ARMOR_OR_BUFF
            capture.open(BUFF_RATH);
        #else
            capture.open(ARMOR_PATH);
        #endif
    #else
        capture.open(CAMERA_PATH);
    #endif

    if(!capture.isOpened()){
        printf("can not open ...\n");
        return ;
    }
    while(1){
        capture>>image;
        //cout<<"image  " <<image.rows<<"  "<<image.cols<<endl;
        ProduceIndex++;
        waitKey(1);
        imshow("image", image);
    }
}
void AimSolve(){
    ArmorDetector A;
    BuffDetector B;
    float AngleX, AngleY,Distance;
    int fd=open_port();
    set_opt(fd,115200,8,'N',1);
    while(1){
        while(ProduceIndex-ConsumptionIndex<IMAGE_CACHE);
        ConsumptionIndex=ProduceIndex;
        if(ARMOR_OR_BUFF){
            B.BuffDetectTask(image);
            B.GetAngle(AngleX, AngleY,Distance);
        }else{
            A.ArmorDetectTask(image);
            A.GetAngle(AngleX, AngleY,Distance);
        }
        if(AngleX!=0 || AngleY!=0 || Distance!=0){

            YawPID.target=targetYaw;
            YawPID.feedback=AngleX;
            ansX=pid_calc(&YawPID);

            PithPID.target=targetPith;
            PithPID.feedback=AngleY;
            ansY=pid_calc(&PithPID);

            cout<<ansX<<"  "<<ansY<<endl;
            buf[0]=55;
            buf[1]=ansX>>8;
            buf[2]=ansX;
            buf[3]=ansY>>8;
            buf[4]=ansY;
            write(fd,buf, 6);
        }
    }
    return ;
}


int main(){
    thread Camera(CameraSolve);
    thread Aim(AimSolve);
    Camera.detach();
    Aim.join();
    return 0;
}
