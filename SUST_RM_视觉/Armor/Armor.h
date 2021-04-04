#ifndef Armor_H
#define Armor_H


#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include "ArmorAngle.h"

using namespace cv;
using namespace std;

class LedStick{
    public:
        LedStick():Matched(false){}
        LedStick(const RotatedRect& R1){
            R.angle=R1.angle;
            R.center=R1.center;
            R.size=R1.size;
            Matched=false;
        }
        RotatedRect R;  // 灯条信息
        bool Matched;   // 匹配状态
        size_t MatchIndex;  // 匹配序号
        float MatchFactor;  // 匹配强度
};

class Armor{
    public:
        Armor(){};
        Armor(const LedStick& L1,const LedStick& L2);
        void DrawRect(Mat& Img,Point2f Roi) const;
        void DrawSpot(Mat& Img, Point2f Roi) const;
        int GetAverageIntensity(const Mat& Img);    //计算装甲板roi平均色彩强度，筛选装甲板中心有灯条
        void MaxMatch(vector<LedStick>& Led, size_t i, size_t j);   //灯条匹配
        bool IsSuitableSize(void) const;
        LedStick Led[2];    // 装甲板的两个灯条
        float ErrorAngle;   // 两个灯条的误差的角度
        Point2i Center; // 装甲板中心点
        Rect2i R;    // 装甲板roi矩形
        int AverageIntensity;   //平均色彩强度
};
Armor::Armor(const LedStick& L1, const LedStick& L2){
    Led[0]= L1;//记录装甲板的两个灯条
    Led[1]= L2;
    ErrorAngle = fabs(L1.R.angle - L2.R.angle);
    R.width = abs(L1.R.center.x - L2.R.center.x);
    R.height = (L1.R.size.height + L1.R.size.height)/2.0;
    Center.x = (L1.R.center.x + L2.R.center.x)/2.0;
    Center.y = (L1.R.center.y + L2.R.center.y)/2.0;
    R.x = Center.x - R.width/3;
    R.y = Center.y - R.height/3;
    R.width*= 2.0/3;
    R.height*= 2.0/3;
}
class ArmorDetector{
    public:
        ArmorDetector(){
            Angle=SolveAngleArmor(CAMERA0_FILEPATH, LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL);
        }
        Rect GetRoi(const Mat &Img);
        bool MakeRectSafe(Rect& rect,Size size);
        bool DetectArmor(Mat& Img, Rect RoiRect);
        bool ArmorDetectTask(Mat& Img);
        void GetAngle(float &X, float &Y, float &Z);
    private:
        Rect LastTarget;
        int LostCnt=0;
        int DetectCnt=0;
    public:
        // 调试参数
        int OffsetX = 100;
        int ShortOffsetY = 100;
        int ColorTh = 10;
        int GrayTh = 60;
        int ColorTh_Red = 10;
        int GrayTh_Red = 60;
        int ColorTh_Blue = 10;
        int GrayTh_Blue = 60;
    private:
        float Distance = 0;
        float AngleX = 0;
        float AngleY = 0;
        vector<Point2f> Points2d;
    private:
        SolveAngleArmor Angle;

};
void Armor::DrawRect( Mat& Img, Point2f Roi) const{
    rectangle(Img, R+Point_<int>(Roi), Scalar(255,255,255), 1);
}

void Armor::DrawSpot(Mat& Img, Point2f Roi) const{
    circle(Img, Center + Point_<int>(Roi), int(R.height/4), Scalar(0,0,255), -1);
}
int Armor::GetAverageIntensity(const Mat& img) {
    if(R.width < 1 || R.height < 1 || R.x < 1 || R.y < 1 || R.width + R.x > img.cols || R.height + R.y > img.rows) return 255;
    Mat roi = img(Range(R.y, R.y + R.height), Range(R.x, R.x + R.width));
    AverageIntensity = mean(roi).val[0];
    return AverageIntensity;
}
void Armor::MaxMatch(vector<LedStick>& Led,size_t i,size_t j){// 灯条匹配算法
    RotatedRect R, L;
    if( Led[0].R.center.x >  Led[1].R.center.x){
        R =  Led[0].R;
        L =  Led[1].R;
    }else{
        R =  Led[1].R;
        L =  Led[0].R;
    }

    float Angle_8 = L.angle - R.angle;
    if(Angle_8 < 1e-3f) Angle_8 = 0.0f;
    float f = ErrorAngle + 0.5 * Angle_8;
    if(!Led.at(i).Matched && !Led.at(j).Matched){

        Led.at(i).Matched = true;
        Led.at(i).MatchIndex = j;
        Led.at(j).Matched = true;
        Led.at(j).MatchIndex = i;
        Led.at(i).MatchFactor = f;
        Led.at(j).MatchFactor = f;
    }
    if(Led.at(i).Matched && !Led.at(j).Matched){
        if(f < Led.at(i).MatchFactor){
            Led.at(Led.at(i).MatchIndex).Matched = false;
            Led.at(i).MatchFactor = f;
            Led.at(i).MatchIndex = j;
            Led.at(j).Matched = true;
            Led.at(j).MatchFactor = f;
            Led.at(j).MatchIndex = i;

        }
    }
    if(Led.at(j).Matched && !Led.at(i).Matched){
        if(f < Led.at(j).MatchFactor ){
            Led.at(Led.at(j).MatchIndex).Matched = false;
            Led.at(j).MatchFactor = f;
            Led.at(j).MatchIndex = i;
            Led.at(i).Matched = true;
            Led.at(i).MatchFactor = f;
            Led.at(i).MatchIndex = j;
        }
    }
    if(Led.at(j).Matched && Led.at(i).Matched &&
            Led.at(i).MatchFactor > f &&
            Led.at(j).MatchFactor > f){
        Led.at(Led.at(j).MatchIndex).Matched = false;
        Led.at(Led.at(i).MatchIndex).Matched = false;
        Led.at(i).Matched = true;
        Led.at(i).MatchFactor = f;
        Led.at(i).MatchIndex = j;
        Led.at(j).Matched = true;
        Led.at(j).MatchFactor = f;
        Led.at(j).MatchIndex = i;
    }
}
bool Armor::IsSuitableSize(void) const  // 判断可能的装甲板是否符合尺寸
{
    if(Led[0].R.size.height*0.7f < Led[1].R.size.height && Led[0].R.size.height*1.3f > Led[1].R.size.height){
        float ArmorWidth=fabs(Led[0].R.center.x - Led[1].R.center.x);
        if(ArmorWidth > Led[0].R.size.width
           && ArmorWidth > Led[1].R.size.width
           && ArmorWidth > (Led[0].R.size.width+Led[1].R.size.width)*3){
            float HMax = (Led[0].R.size.height + Led[1].R.size.height)/2.0f;
            if(fabs(Led[0].R.center.y - Led[1].R.center.y) < 0.8f* HMax ){
                if(HMax*4.0f > R.width && HMax < 1.2f* R.width){
                    return true;
                }
            }
        }
    }
    return false;
}
Rect ArmorDetector::GetRoi(const Mat &Img){
    Size ImgSize=Img.size();
    Rect RectTmp=LastTarget;
    Rect RectRoi;
    if(RectTmp.x==0||RectTmp.y==0||
       RectTmp.width==0||RectTmp.height==0||
       LostCnt>=15||DetectCnt%100==0){
        LastTarget=Rect(0,0,ImgSize.width,ImgSize.height);
        RectRoi=Rect(0,0,ImgSize.width,ImgSize.height);
        return RectRoi;
    }else{
        float Scale=2;
        if(LostCnt<30) Scale=3;
        else if(LostCnt<=60) Scale=4;
        else if(LostCnt<=120) Scale=5;

        int w=int(RectTmp.width*Scale);
        int h=int(RectTmp.height*Scale);
        int x=min(max(int(RectTmp.x-(w-RectTmp.width)*0.5f),0),ImgSize.width);
        int y=min(max(int(RectTmp.y-(h-RectTmp.height)*0.5f),0),ImgSize.height);
        w=min(int(RectTmp.width*Scale),ImgSize.width-x);
        h=min(int(RectTmp.height*Scale),ImgSize.height-y);
        RectRoi=Rect(x,y,w,h);
    }
    return RectRoi;
}
bool ArmorDetector::DetectArmor(Mat& Img, Rect RoiRect){
    Mat RoiImage = Img(RoiRect);
    Point2f OffsetRoiPoint(RoiRect.x, RoiRect.y);
    vector<LedStick> Led;
    Mat BinaryBrightnessImg, BinaryColorImg, Gray;
    cvtColor(RoiImage,Gray,COLOR_BGR2GRAY);
    vector<Mat> Bgr;
    split(RoiImage, Bgr);
    Mat ResultImg;
    if(RED_OR_BLUE) subtract(Bgr[2], Bgr[1], ResultImg);//图像的减法操作
    else subtract(Bgr[0], Bgr[1], ResultImg);
    threshold(Gray, BinaryBrightnessImg, GrayTh, 255, CV_THRESH_BINARY);//阈值化得到二值图
    threshold(ResultImg, BinaryColorImg, ColorTh, 255, CV_THRESH_BINARY);
    vector<vector<Point>> ContoursLight;
    vector<vector<Point>> ContoursBrightness;
    findContours(BinaryColorImg, ContoursLight, RETR_EXTERNAL, CHAIN_APPROX_NONE);//轮廓提取
    findContours(BinaryBrightnessImg, ContoursBrightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    imshow("BinaryColorImg", BinaryColorImg);
    for(size_t i = 0; i < ContoursBrightness.size(); i++){
        double Area = contourArea(ContoursBrightness[i]); //计算轮廓面积
        if (Area < 20.0 || 1e5 < Area) continue;
        for(size_t ii = 0; ii < ContoursLight.size(); ii++){
            if(pointPolygonTest(ContoursLight[ii], ContoursBrightness[i][0], false) >= 0.0 ){
                double length = arcLength(ContoursBrightness[i], true); // 返回轮廓的周长
                if (length > 15 && length <4000){
                    RotatedRect RRect = fitEllipse(ContoursBrightness[i]);//RotatedRect 参数 中点，边长，旋转角度
                    // 旋转矩形提取四个点
                    Point2f RectPoint[4];
                    RRect.points(RectPoint);
                    for (int j = 0; j < 4 ; j++){
                        line(Img, RectPoint[j]+OffsetRoiPoint, RectPoint[(j+1)%4]+OffsetRoiPoint, Scalar(255,0,255),1);
                    }
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f) RRect.angle =  RRect.angle - 180.0f;
                    if (fabs(RRect.angle) <= 30){
                        LedStick r(RRect);
                        Led.push_back(r);
                    }
                }
                break;
            }
        }
    }


    for(size_t i = 0; i < Led.size() ; i++){
        for(size_t j = i + 1; j < Led.size() ; j++){
            Armor ArmTmp(Led.at(i), Led.at(j));
            if (ArmTmp.ErrorAngle < 8.0f){
                putText(Img, to_string(ArmTmp.R.width/(ArmTmp.R.height+0.0001)), ArmTmp.Center + Point_<int>(OffsetRoiPoint) , FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
                if(ArmTmp.IsSuitableSize()){
                    if(ArmTmp.GetAverageIntensity(Gray)< 50 ){
                        ArmTmp.MaxMatch(Led, i, j);
                    }
                }
            }
        }
    }
    vector<Armor> FinalArmorList;
    for(size_t i = 0; i < Led.size() ; i++){
        if(Led.at(i).Matched){
            Led.at(Led.at(i).MatchIndex).Matched = false;
            Armor ArmTmp( Led.at(i), Led.at(Led.at(i).MatchIndex));
            FinalArmorList.push_back(ArmTmp);
        }
    }

    float Dist=1e8;
    bool FoundFlag = false;
    Armor Target;
    Point2f RoiCenter(RoiRect.width/2, RoiRect.height/2);
    float dx,dy;
    for (size_t i = 0; i < FinalArmorList.size() ; i++ ){
        dx = fabs(FinalArmorList.at(i).Center.x - RoiCenter.x);
        dy = fabs(FinalArmorList.at(i).Center.y - RoiCenter.y);
        if( dx + dy < Dist){
            Target = FinalArmorList.at(i);
            Dist = dx + dy;
        }
        FinalArmorList.at(i).DrawRect(Img, OffsetRoiPoint);
        FoundFlag = true;
    }
    rectangle(Img, RoiRect,Scalar(255, 0, 255),1);


    RotatedRect TargetRect;
    if(FoundFlag){
        //cout<<OffsetRoiPoint.x<<"  "<<OffsetRoiPoint.y<<endl;
        AngleX=OffsetRoiPoint.x;
        AngleY=OffsetRoiPoint.y;
        Target.DrawSpot(Img, OffsetRoiPoint);
        Point2f PointTmp[4];
        Point2f Point2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if(Target.Led[0].R.center.x > Target.Led[1].R.center.x){
            R = Target.Led[0].R;
            L = Target.Led[1].R;
        }else{
            R = Target.Led[1].R;
            L = Target.Led[0].R;
        }
        L.points(PointTmp);
        Point2d[0] = PointTmp[1];
        Point2d[3] = PointTmp[0];
        R.points(PointTmp);
        Point2d[1] = PointTmp[2];
        Point2d[2] = PointTmp[3];
        // 计算补偿，用于调试调整准心
        Point2f OffsetPoint;
        OffsetPoint = Point2f(100, 100) - Point2f(OffsetX,ShortOffsetY);


        Points2d.clear();
        vector<Point2f> points_roi_tmp;
        for(int i=0;i<4;i++){
            points_roi_tmp.push_back(Point2d[i] + OffsetRoiPoint);
            Points2d.push_back(Point2d[i] + OffsetRoiPoint +OffsetPoint);
        }
        LastTarget = boundingRect(points_roi_tmp);
        rectangle(Img, LastTarget,Scalar(255,255,255), 1);
        LostCnt = 0;
    }else{
        LostCnt ++;
    }
    DetectCnt++;
    return FoundFlag;
}
bool ArmorDetector::ArmorDetectTask(Mat& Img){
    Rect Roi = GetRoi(Img);
    if(DetectArmor(Img, Roi)){
        //Angle.Generate3DPoints((uint)1, Point2f());
        //Angle.getAngle(Points2d, 15,AngleX,AngleY,Distance);
        if(Distance > 5000)
            return false;
        return true;
    }else{
        AngleX = 0;
        AngleY = 0;
        Distance = 0;
        return false;
    }
}
void ArmorDetector::GetAngle(float &X, float &Y, float &Z)
{
    X = AngleX;
    Y = AngleY;
    Z = Distance;
}

#endif
