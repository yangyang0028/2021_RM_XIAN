#include "iostream"
#include "opencv2/opencv.hpp"
#include "BuffAngle.h"

using namespace std;
using namespace cv;

#define BUFF_OFFSET_x 101
#define BUFF_OFFSET_y 118
#define BULLET_SPEED 28.5

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;

class BuffDetector{
public:
    BuffDetector(){
        Angle=SolveAngleBuFF(CAMERA0_FILEPATH, LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL);
    }
    bool BuffDetectTask(Mat& img);
    void GetAngle(float &X, float &Y,float &Z);
private:
    bool DetectBuff(Mat& img);
public:
    int BuffOffsetX = BUFF_OFFSET_x;
    int BuffOffsetY = BUFF_OFFSET_y;
    int BeginOffsetX = BUFF_OFFSET_x;
    int BeginOffsetY = BUFF_OFFSET_y;
    int WorldOffsetX=750;
    int ColorTh=16;
    float BuffAngle = 0;
    float DiffAngle = 0;
    int AreaRatio = 500;
private:
    float AngleX = 0;
    float AngleY = 0;
    float Distance=0;
    vector<Point2f> Points2d;
private:
    SolveAngleBuFF Angle;
};
class Object{
public:
    void DrawTarget(Mat &img){
        if(Type == INACTION)
            circle(img, SmallRect.center, 3, Scalar(0, 0, 255), -1);
        else if(Type == ACTION)
            circle(img, SmallRect.center, 3, Scalar(255, 255, 255), -1);
        else
            circle(img, SmallRect.center, 3, Scalar(255, 255, 255), 1);
    }
    void UpdateOrder();    // 更新能量机关装甲板的绝对位置
    void KnowYourself(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）
    RotatedRect SmallRect;    // 能量机关扇叶内轮廓
    RotatedRect BigRect;  // 能量机关扇叶外轮廓
    vector<Point2f> Points2d; // ｐｎｐ角度解算的四个点
    float Angle;
    float DiffAngle;
    int Type = UNKOWN;
};

bool BuffDetector::DetectBuff(Mat& img){
    GaussianBlur(img, img, Size(3,3),0);
    Points2d.clear();
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;
    subtract(bgr[2], bgr[1], result_img);
    Mat binary_color_img;
    threshold(result_img, binary_color_img, ColorTh, 255, CV_THRESH_BINARY);
    vector<Object> vec_target;
    vector<Rect> vec_color_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_color_img,contours,hierarchy,CV_RETR_CCOMP,CHAIN_APPROX_NONE);
    for(size_t i=0; i < contours.size();i++){

        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;
        // 小轮廓面积条件
        double SmallRectarea = contourArea(contours[i]);
        double SmallRectlength = arcLength(contours[i],true);
        if(SmallRectlength < 10)
            continue;
        // 用于超预测时比例扩展时矩形的判断
        Rect rect = boundingRect(contours[static_cast<uint>(hierarchy[i][3])]);
        vec_color_rect.push_back(rect);

        if(SmallRectarea < 200)
            continue;
        // 大轮廓面积条件
        double BigRectarea = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double BigRectlength = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
        if(BigRectarea < 300)
            continue;
        if(BigRectlength < 50)
            continue;
        // 能量机关扇叶进行拟合
        Object object;

        object.SmallRect=fitEllipse(contours[i]);
        object.BigRect = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);

        Point2f small_point_tmp[4];
        object.SmallRect.points(small_point_tmp);
        Point2f big_point_tmp[4];
        object.BigRect.points(big_point_tmp);
        for(int k=0;k<4;k++)
        {
            line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 1);
            line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
        }
        object.DiffAngle=fabsf(object.BigRect.angle-object.SmallRect.angle);

        if(object.SmallRect.size.height/object.SmallRect.size.width < 3)
        {
            if(object.DiffAngle<100 && object.DiffAngle>80)
            {

                float SmallRectsize_ratio;
                SmallRectsize_ratio = object.SmallRect.size.height/object.SmallRect.size.width;

                // 根据轮廓面积进行判断扇叶类型
                double area_ratio = AreaRatio/100;
                if(SmallRectarea * 12 >BigRectarea && SmallRectarea* area_ratio<BigRectarea
                        && SmallRectsize_ratio > 1 && SmallRectsize_ratio < 3.0f)
                {
                    object.Type = ACTION;  // 已经激活类型
                }else if(SmallRectarea * area_ratio>=BigRectarea && SmallRectarea *2 < BigRectarea
                         && SmallRectsize_ratio > 1 && SmallRectsize_ratio < 3.0f)
                {
                    // 更新世界坐标系顺序
                    object.Type = INACTION;    // 未激活类型
                    object.UpdateOrder();
                    //object.KnowYourself(binary_color_img);
                    vec_target.push_back(object);
                }else
                {
                    object.Type = UNKOWN;    // 其他类型
                }
                double multiple_area=fabs(BigRectarea/SmallRectarea);
                putText(img, to_string(multiple_area), Point2f(50,50)+ object.SmallRect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
            }
        }
    }
    // 遍历所有结果并处理\选择需要击打的目标
    Object final_target;
    bool find_flag = false;
    float DiffAngle = 1e8;
    for(size_t i=0; i < vec_target.size(); i++)
    {
        Object object_tmp = vec_target.at(i);
        if(object_tmp.Type == INACTION){
            find_flag = true;
            float ang = fabs(vec_target[i].DiffAngle-90.0f);
            if(ang < DiffAngle)
            {
                final_target = vec_target.at(i);
                DiffAngle = ang;
            }
            Point2f buff_offset = Point2f(BuffOffsetX - 100, 100 - BuffOffsetY);
            vector<Point2f> vec_Points2dtmp;
            for(size_t k=0; k < 4; k++)
            {
                vec_Points2dtmp.push_back(final_target.Points2d.at(k) + buff_offset);
            }
            Points2d = vec_Points2dtmp;
            BuffAngle = final_target.Angle;

        }
    }
    if(find_flag){
        final_target.DrawTarget(img);
    }
    return find_flag;
}
double Point_distance(Point2f p1,Point2f p2)
{
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}
void Object::UpdateOrder()
{
    Points2d.clear();
    Point2f points[4];
    SmallRect.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = Point_distance(point_up_center, BigRect.center);
    double down_distance = Point_distance(point_down_center, BigRect.center);
    if(up_distance > down_distance){
        Angle = SmallRect.angle;
        Points2d.push_back(points[0]);Points2d.push_back(points[1]);
        Points2d.push_back(points[2]);Points2d.push_back(points[3]);
    }else{
        Angle = SmallRect.angle + 180;
        Points2d.push_back(points[2]);Points2d.push_back(points[3]);
        Points2d.push_back(points[0]);Points2d.push_back(points[1]);
    }
}
int GetRectIntensity(const Mat &img, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}
void Object::KnowYourself(Mat &img)
{
    Point2f vector_height = Points2d.at(0) - Points2d.at(3);
    Point left_center = Points2d.at(3) - vector_height;
    Point right_center = Points2d.at(2) - vector_height;
    int width = 5;
    int height = 5;
    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);
    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);
    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);
    int left_intensity = GetRectIntensity(img, left_rect);
    int right_intensity = GetRectIntensity(img, right_rect);
    if(left_intensity > 10 && right_intensity > 10)
    {
        Type = ACTION;
    }else{
        Type = INACTION;
    }
    putText(img, to_string(left_intensity), left_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
    putText(img, to_string(right_intensity), right_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
}
bool BuffDetector::BuffDetectTask(Mat& img){
    if(DetectBuff(img)){
        Point2f world_offset;
        float world_offset_x = WorldOffsetX - 500;
        float world_offset_y = 800-pow((640000 - pow(world_offset_x, 2)), 0.5);
        float pre_angle;
        world_offset = Point2f(-world_offset_x, -world_offset_y);
        pre_angle = atan(world_offset_x/(800-world_offset_y));

        Angle.Generate3DPoints((uint)2, world_offset);
        Angle.getBuffAngle(Points2d, BULLET_SPEED
                            , BuffAngle, pre_angle
                            , AngleX, AngleY, Distance);
        return true;
    }else{
        AngleX = 0;
        AngleY = 0;
        Distance = 0;
        return false;
    }
}
void BuffDetector::GetAngle(float &X, float &Y, float &Z)
{
    X = AngleX;
    Y = AngleY;
    Z = Distance;
}
