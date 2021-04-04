#ifndef ArmorAngle_h
#define ArmorAngle_h
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include "../Base.h"
using namespace std;
using namespace cv;

class SolveAngleArmor{
    public:
        SolveAngleArmor(){}
        SolveAngleArmor(const char* file_path, float c_x, float c_y, float c_z, float barrel_y);
        void getAngle(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist);
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

};
SolveAngleArmor::SolveAngleArmor(const char* file_path, float c_x, float c_y, float c_z, float barrel_y)
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
void SolveAngleArmor::Generate3DPoints(uint mode, Point2f offset_point){
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
void SolveAngleArmor::getAngle(vector<Point2f> &image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist){
    // 姿态结算
    solvePnP(ObjectPoints, image_point, CameraMatrix, DistCoeffs, Rvec, Tvec);
    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(PtzCameraY + BarrelPtzOffsetY))/static_cast<double>(OverlapDist);
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(PtzCameraX),static_cast<double>(PtzCameraY),static_cast<double>(PtzCameraZ)};
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat position_in_ptz;
    position_in_ptz = r_camera_ptz * Tvec - t_camera_ptz;

    //计算子弹下坠补偿
    double bullet_speed = static_cast<double>( ballet_speed);
    const double *_xyz = (const double *)position_in_ptz.data;
    double down_t = 0.0;
    if(bullet_speed > 10e-3)
        down_t = _xyz[2] /1000.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t*down_t * 1000;

    // 计算角度
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    if(BarrelPtzOffsetY != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(BarrelPtzOffsetY)/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));

        if(xyz[1] < 0)
        {
            Beta = atan(-xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha+Beta)); //camera coordinate
        }else if(xyz[1] < static_cast<double>(BarrelPtzOffsetY))
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha - Beta));
        }else
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>((Beta-alpha));   // camera coordinate
        }
    }else{
        angle_y = static_cast<float>(atan2(xyz[1],xyz[2]));
    }
    if(BarrelPtzOffsetX != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(BarrelPtzOffsetX)/sqrt(xyz[0]*xyz[0] + xyz[2]*xyz[2]));
        if(xyz[0] > 0)
        {
            Beta = atan(-xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha+Beta)); //camera coordinate
        }else if(xyz[0] < static_cast<double>(BarrelPtzOffsetX))
        {
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha - Beta));
        }else{
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(Beta-alpha);   // camera coordinate
        }
    }else{
        angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
    }
    angle_x = static_cast<float>(angle_x) * 57.2957805f;
    angle_y = static_cast<float>(angle_y) * 57.2957805f;
    dist = static_cast<float>(xyz[2]);
}
#endif
