#pragma once
//stdlib
#include <stdlib.h>  
#include <iostream>  
#include <string>  
//OpenNI  
#include <XnCppWrapper.h>  //OpenNI c++ 头文件
//OpenCV
#include "opencv/cv.h"  
#include "opencv/highgui.h"  
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace std;  
using namespace cv; 

struct Camera
{
  double cx, cy, fx, fy, depthScale;
            
};

class Kinectreader
{
private:
  XnStatus result;// = XN_STATUS_OK;    
  xn::DepthMetaData depthMD;  //depth metadata
  xn::ImageMetaData imageMD;  //image metadata
  
  //OpenCV  
  IplImage*  imgDepth16u;//;=cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);  
  IplImage* imgRGB8u;//=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
  IplImage*  depthShow;//=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);  
  IplImage* imageShow;//=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);    
  
  // context   
  xn::Context context; 
  xn::DepthGenerator depthGenerator; 
  xn::ImageGenerator imageGenerator;
  XnMapOutputMode mapMode; 
  
  //相机外参
  Eigen::Matrix4d Tcm;//相机坐标系向移动机器人坐标系的变换矩阵
  
  //相机模型
  Camera camera;
    pcl::PassThrough<pcl::PointXYZ> pass; //直通滤波器,截取点云区域
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//统计离群点滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel; //网格滤波器，调整点云地图分辨率
    int MeanK;//统计滤波范围点
    double zmin,zmax,voxel_grid;//直通滤波器z方向区域,网格滤波分辨率
    double outlier_threshold;//离群点阈值
  
public:
  Kinectreader();
  void Init();
  IplImage* ReadKinect();
  void StopRead();
  void CheckOpenNIError(XnStatus result, std::string status);
  void ShowDepthAndImg();
  pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointCloud();
  
  
  
};
