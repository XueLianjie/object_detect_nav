#include <stdlib.h>  
#include <iostream>  
#include <string>  
//OpenNI  
#include <XnCppWrapper.h>  //OpenNI c++ 头文件
//OpenCV
#include "opencv/cv.h"  
#include "opencv/highgui.h"  
//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/people/ground_based_people_detection_app.h>//这都有？？


#include "kinectreader.h"
//#include "config.h"   
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_no_plane,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_plane);

int main( int argc, char** argv )  
{  
  pcl::PCDWriter writer;
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pclvisualizer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pclvisualizer->initCameraParameters ();//Initialize camera parameters with some default values. 
  //pclvisualizer->addCoordinateSystem(1.0);
    
  int v1(0);
  pclvisualizer->createViewPort(0.0,0.0,0.5,1.0,v1);
  pclvisualizer->setBackgroundColor(150,150,150,v1);
  pclvisualizer->addCoordinateSystem(1.0, v1);
  int v2(1);
  pclvisualizer->createViewPort(0.5,0.0,1.0,1.0,v2);
  pclvisualizer->setBackgroundColor(150,150,150,v2);
  pclvisualizer->addCoordinateSystem(1.0, v2);
  
  //pcl::visualization::CloudViewer viewer("cloud");//点云显示
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;//点提取对象**需要包含#include<pcl/filters/extract_indices.h>
  pcl::ExtractIndices<pcl::Normal> extract_normals;//点提取对象
  
  
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients); //coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  
  
  char key=0,times=0; 
  Kinectreader reader;
  reader.Init();
  while(key != 27)
  {
    reader.ReadKinect();
    //reader.ShowDepthAndImg();
    pPointCloud = reader.ToPointCloud();	    
    
    //点云分割
    /*
     * 先计算点云的法向量
     * 然后进行分割，获得内点索引，和拟合模型参数
     * 提取模型内点到新的点云对象中
     * 创建投影对象，将去除平面的点云向平面上投影，获得投影后的点云对象
     */
    
    ne.setSearchMethod(tree);
    ne.setInputCloud(pPointCloud);
    ne.setKSearch(40);
    ne.compute(*cloud_normals);
    
    //设置分割所用的模型类型、方法和相关参数
    seg.setOptimizeCoefficients(true);//设置为false 后点云出现略微抖动，不太稳定
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.08);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.06);
    seg.setInputCloud(pPointCloud);
    seg.setInputNormals(cloud_normals);
    //执行分割，获取模型参数和内点
    seg.segment(*inliers_plane, *coefficients_plane);
    
    Eigen::Vector3f plane_coe_vec(coefficients_plane->values[0],coefficients_plane->values[1],coefficients_plane->values[2]);
    
    //std::cerr << "Plane coefficients: " << plane_coe_vec << std::endl;
    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    extract.setInputCloud(pPointCloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);//是否显示物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_no_plane);
    extract.setNegative(false);//是否显示物体
    extract.filter(*cloud_plane);
    //projection
    
  
    std::vector<pcl::PointIndices> cluster_indices;//存放内点索引的容器
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.03); // 3cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_no_plane);
    ec.extract (cluster_indices);
    

    pclvisualizer->removeAllPointClouds(0);//viewport = 0
    pclvisualizer->removeAllShapes(0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(cloud_plane, 255, 0, 0);//红色地面点云
    pclvisualizer->addPointCloud<pcl::PointXYZ> (cloud_plane, single_color_red, "plane");//添加红色地面点云

    
    unsigned int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      //将聚类的点云存储
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	cloud_cluster->points.push_back (cloud_no_plane->points[*pit]); 
      
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      
      //投影到地面上
      pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::ProjectInliers<pcl::PointXYZ> projection;
      projection.setModelType(pcl::SACMODEL_PLANE);
      projection.setInputCloud(cloud_cluster);
      
      projection.setModelCoefficients(coefficients_plane);
      
      projection.filter(*projected_cluster);
      
      
      //* this is for ransac 
      //pcl::SampleConsensus::
      //pcl::ExtractIndices<pcl::PointXYZ> out_line_extracter;//点提取对象**需要包含#include<pcl/filters/extract_indices.h>
      pcl::PointCloud<pcl::PointXYZ>::Ptr line_inliers1(new pcl::PointCloud<pcl::PointXYZ>),vertex_inliers(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<int> inliers1,inliers2;
      std::vector<double> distances1,distances2;
      pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (projected_cluster));
      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
      ransac.setDistanceThreshold(0.003);
      ransac.computeModel();
      ransac.getInliers(inliers1);
      Eigen::Vector2d point_inline1(ransac.model_coefficients_[0],ransac.model_coefficients_[2]),point_inline2;//直线上的点的坐标
      
      Eigen::Vector2d direction_vec1(ransac.model_coefficients_[3],ransac.model_coefficients_[5]);//直线的方向向量
      //Eigen::Vector2d direction_vec2 = direction_vec1.cross(plane_coe_vec);//外积为第二条边的方向向量
      Eigen::Vector2d direction_vec2(-direction_vec1(1), direction_vec1(0));
      
      Eigen::Vector2d tmp_point,base_point,vec,best_point, s_point, best_point1, best_point2;
      double dis = 0.0,best_dist1 = 0.0,best_dist2=0.0, xs1 = 0.0, zs1 = 0.0, xs2, zs2, xs12, zs12, xs21, zs21, k1 = 0.0,k2 = 0.0,dist1 = 0.0,dist2 = 0.0;
      long int num = 0,best_num = 0,r=0;
      for(int i = 0; i < 200; i++)
      {
	r = rand()%projected_cluster->points.size();
	base_point(0) = projected_cluster->points[r].x;//索引一定要写成0，1
	base_point(1) = projected_cluster->points[r].z;
	for(int j = 0; j < projected_cluster->points.size(); j++)
	{
	  tmp_point(0) = projected_cluster->points[j].x;
	  tmp_point(1) = projected_cluster->points[j].z;
	  //tmp_point(projected_cluster->points[j].x, projected_cluster->points[j].z);
	  vec = tmp_point - base_point;
	  dis = vec.dot(direction_vec1);
	  if(abs(dis) < 0.002)
	    num ++; 
	  
	}
	if (num > best_num)
	{
	  best_point = base_point;
	  best_num = num;
	  
	}
	num = 0;
      }
      point_inline2 = best_point;
      k1 = direction_vec1(1)/direction_vec1(0);
      k2 = direction_vec2(1)/direction_vec2(0);
      xs1 = -(point_inline1(1)-point_inline2(1) - k1 * point_inline1(0) + k2 * point_inline2(0))/(k1-k2);
      zs1 = - (k2 * point_inline1(1) - k1 * point_inline2(1) - k1*k2*point_inline1(0) + k1 * k2 * point_inline2(0))/(k1 - k2);
      s_point(0) = xs1;
      s_point(1) = zs1;
      for(int k = 0; k < projected_cluster->points.size(); k++)
      {
	tmp_point(0) = projected_cluster->points[k].x;
	tmp_point(1) = projected_cluster->points[k].z;
	vec = tmp_point - s_point;
	dist1 = vec.dot(direction_vec2);//距离长边的距离
	dist2 = vec.dot(direction_vec1);//离短边的距离
	if(abs(dist1) > best_dist1)
	{
	  best_dist1 = abs(dist1);
	  best_point1 = tmp_point;
	}
	if(abs(dist2) > best_dist2)
	{
	  best_dist2 = abs(dist2);
	  best_point2 = tmp_point;
	}
	  
      }
      xs2 = -(best_point1(1)-best_point2(1) - k1 * best_point1(0) + k2 * best_point2(0))/(k1-k2);
      zs2 = - (k2 * best_point1(1) - k1 * best_point2(1) - k1*k2*best_point1(0) + k1 * k2 * best_point2(0))/(k1 - k2);
      
      xs12 = -(point_inline1(1)-best_point2(1) - k1 * point_inline1(0) + k2 * best_point2(0))/(k1-k2);
      zs12 = - (k2 * point_inline1(1) - k1 * best_point2(1) - k1*k2*point_inline1(0) + k1 * k2 * best_point2(0))/(k1 - k2);
      
      xs21 = -(best_point1(1)-point_inline2(1) - k1 * best_point1(0) + k2 * point_inline2(0))/(k1-k2);
      zs21 = - (k2 * best_point1(1) - k1 * point_inline2(1) - k1*k2*best_point1(0) + k1 * k2 * point_inline2(0))/(k1 - k2);
      
      vertex_inliers->resize(4);
      vertex_inliers->points[0].x = xs1;
      vertex_inliers->points[0].z = zs1;
      vertex_inliers->points[1].x = xs12;
      vertex_inliers->points[1].z = zs12;
      vertex_inliers->points[2].x = xs21;
      vertex_inliers->points[2].z = zs21;
      vertex_inliers->points[3].x = xs2;
      vertex_inliers->points[3].z = zs2;
      
      
      model_l->getDistancesToModel(ransac.model_coefficients_, distances1);
      std::vector<double>::iterator max_distance1_iter = max_element(distances1.begin(),distances1.end());
      double max_distance = *max_distance1_iter;
      
      /*pcl::copyPointCloud<pcl::PointXYZ>(*projected_cluster, inliers1, *line_inliers1);
      //先计算外点，然后计算交点
      //Eigen::Vector3f point_iter = projected_cluster->points[1];
      //model_l->selectWithinDistance (ransac.model_coefficients_, 0.3, inliers2);
      //std::cout << ransac.model_coefficients_ << std::endl;
      */
      
      //显示和存储
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::cout << "width: " << best_dist1 << " m" << std::endl;
      std::cout << "longth: " << best_dist2 << " m" << std::endl;
      std::stringstream ss1,ss2,ss3;
      ss1 << "cloud_cluster_" << j << ".pcd";
      ss2 << "cloud_projected_cluster_" << j << ".pcd";
      ss3 << "cluster" << j;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_cluster(cloud_cluster, j*100%256, j*200%256, (j+200)*300%256);//设置地面上物体点云为纯绿色
      pclvisualizer->addPointCloud<pcl::PointXYZ> (cloud_cluster, single_color_cluster, ss1.str(),v1);//添加绿色点云并设置ID号为“no_plane”
      pclvisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss1.str(),v1);//地面上物体点云设置大小为2

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_projected_cluster(projected_cluster, j*100%256, j*200%256, (j+200)*300%256);//设置地面上物体点云为纯绿色
      pclvisualizer->addPointCloud<pcl::PointXYZ> (projected_cluster, single_color_projected_cluster, ss2.str(),v2);//添加绿色点云并设置ID号为“no_plane”
      pclvisualizer->addLine(vertex_inliers->points[0],vertex_inliers->points[1],0,255,0,ss3.str() + "line1",v2);
      pclvisualizer->addLine(vertex_inliers->points[0],vertex_inliers->points[2],0,255,0,ss3.str() + "line2",v2);
      pclvisualizer->addLine(vertex_inliers->points[2],vertex_inliers->points[3],0,255,0,ss3.str() + "line4",v2);
      pclvisualizer->addLine(vertex_inliers->points[3],vertex_inliers->points[1],0,255,0,ss3.str() + "line3",v2);
      pclvisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss2.str(),v2);//地面上物体点云设置大小为2
      
      
      //writer.write<pcl::PointXYZ> (ss1.str(), *cloud_cluster, false);
      //writer.write<pcl::PointXYZ> (ss2.str(), *projected_cluster, false);
      //*
      j++;
      
    }
    pclvisualizer->spinOnce(100);//这一句必须得加，以更新使得视窗可以读取数据和显示
    
    /*
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_green(cloud_no_plane, 0, 255, 0);
    pclvisualizer->updatePointCloud<pcl::PointXYZ> (cloud_no_plane, single_color_green, "no_plane");//自动移除id为“sample cloud”的点云，并添加新的点云
    //pclvisualizer->spinOnce (100);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_red(cloud_plane, 255, 0, 0);
    pclvisualizer->updatePointCloud<pcl::PointXYZ> (cloud_plane, single_color_red, "plane");
    pclvisualizer->spinOnce(100);//这一句必须得加，以更新使得视窗可以读取数据和显示
    key=cvWaitKey(20);//必须得加上此句，来让深度图和彩图顺利显示
    */
  }
  reader.StopRead();
  
  return 0; 
}

/*******ransac_demo2_method*******/


































