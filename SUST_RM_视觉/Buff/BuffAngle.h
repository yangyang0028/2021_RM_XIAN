#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include "../Base.h"
using namespace std;
using namespace cv;
class SolveAngleBuFF{
    public:
        SolveAngleBuFF(){}
        SolveAngleBuFF(const char* file_path, float c_x, float c_y, float c_z, float barrel_y);
        void getBuffAngle(vector<Point2f> &image_point, float ballet_speed
                                      , float buff_angle, float pre_angle
                                      , float &angle_x, float &angle_y, float &dist);
        float getBuffPitch(float dist, float tvec_y, float ballet_speed);
        void Generate3DPoints(uint mode, Point2f offset_point);
        Mat CameraMatrix, DistCoeffs;
        Mat ObjectPointMat;
        vector<Point3f> ObjectPoints;//世界坐标系
        Mat Rvec;   //旋转向量世界 旋转到 相机
        Mat Tvec;   //平移向量世界 旋转到 相机
        float OverlapDist = 100000.0;
        float BarrelPtzOffsetX = -0;    //记录枪口和镜头的相对位置
        float BarrelPtzOffsetY = -0;
        float PtzCameraX = 0;   //Camera世界位置
        float PtzCameraY = 52.5;
        float PtzCameraZ = -135;
        public:
            float BuffH;
};
SolveAngleBuFF::SolveAngleBuFF(const char* file_path, float c_x, float c_y, float c_z, float barrel_y)
{
    //读取摄像头标定xml文件
    FileStorage fs(file_path, FileStorage::READ);
    // 相关坐标转换偏移数据
    BarrelPtzOffsetY = barrel_y;
    PtzCameraX = c_x;       // +left
    PtzCameraY = c_y;       // + camera is  ptz
    PtzCameraZ = c_z;//-225;     // - camera is front ptz
    // 读取相机内参和畸变矩阵
    fs["Camera_Matrix"] >> CameraMatrix;
    fs["Distortion_Coefficients"] >> DistCoeffs;
    Generate3DPoints(0,Point2f(0,0));
    Mat(ObjectPoints).convertTo(ObjectPointMat, CV_32F);
    Mat Rvec(3, 1, DataType<double>::type);
    Mat Tvec(3, 1, DataType<double>::type);
}
void SolveAngleBuFF::Generate3DPoints(uint mode, Point2f offset_point){
    ObjectPoints.clear();
    float x, y, z, width = 0.0, height = 0.0;
    switch (mode){
    case 1:
        width = 140;
        height = 60;
        break;
    case 2:
        width = 230;
        height = 130;
        break;
    }
    x = -width/2; y = -height/2; z = 0;
    ObjectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = width/2; y = -height/2; z = 0;
    ObjectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = width/2; y = height/2; z = 0;
    ObjectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = -width/2; y = height/2; z = 0;
    ObjectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
}
void SolveAngleBuFF::getBuffAngle(vector<Point2f> &image_point, float ballet_speed
                              , float buff_angle, float pre_angle
                              , float &angle_x, float &angle_y, float &dist)
{
    // 姿态结算
    solvePnP(ObjectPoints, image_point, CameraMatrix, DistCoeffs, Rvec, Tvec);
    // 距离解算 参考能量机关尺寸
    float H = BUFF_H;   // 大能量机关最底部装甲板桥面地面高度
    float h = 430;      // 步兵枪口距离桥面高度mm
    float D = BUFF_DISTANCE;    //步兵距离能量机关水平距离
    float delta_h = H - h;
    float predict_buff_angle = buff_angle + pre_angle;
    BuffH = 800*sin(predict_buff_angle *3.14/180)+800;   // 计算风车相对最底面装甲高度　０－１６００
    float target_h = delta_h + BuffH;
    dist = sqrt(pow(target_h, 2) + pow(D, 2));

    Tvec.at<double>(2,0) = dist;

    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(PtzCameraY + BarrelPtzOffsetY))/static_cast<double>(OverlapDist);
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(PtzCameraX),static_cast<double>(PtzCameraY),static_cast<double>(PtzCameraZ)};
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat position_in_ptz;
    position_in_ptz = Tvec - t_camera_ptz;

    const double *_xyz = (const double *)position_in_ptz.data;

    // 计算角度
    double xyz[3] = {_xyz[0], _xyz[1], dist};

    angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
    angle_x = static_cast<float>(angle_x) * 57.2957805f;

    float thta = -static_cast<float>(atan2(xyz[1],dist)); // 云台与目标点的相对角度
    float balta = static_cast<float>(atan2(target_h,dist)) - thta; // 云台与地面的相对角度

    angle_y = -getBuffPitch(dist/1000, (target_h)/1000, ballet_speed);
    angle_y += balta;
    angle_y = static_cast<float>(angle_y) * 57.2957805f ;
}
float SolveAngleBuFF::getBuffPitch(float dist, float tvec_y, float ballet_speed)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) {
        // 计算枪口抬升角度
        a = (float) atan2(y_temp, dist);
        // 计算实际落点
        float t;
        t = dist / (ballet_speed * cos(a));
        y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 0.01) {
            break;
        }
    }
    return a;
}
