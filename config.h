#pragma once
#include<stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <string>
#include <boost/concept_check.hpp>
#include <list>
#include <vector>
#include <memory>
#include <set>
#include <map>
#include <unordered_map>
//using namespace std;

//Opencv library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

class Config
{
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config(){}; //私有构造函数无法初始化，可以形成单例模式singleton
public:
  ~Config();
  
  //set a new config file
  static void setParameterFile(const std::string &filename);//静态函数
  
  //access the parameter values. 定义为模板函数以接受不同类型的数据参数
  template <typename T>
  static T get(const std::string &key)//静态成员函数， 不能通过对象调用静态成员函数；  由于静态成员函数不与特定的对象关联， 因此只能使用静态数据成员。 
  {
    return T(Config::config_->file_[key]);
  }
  
};