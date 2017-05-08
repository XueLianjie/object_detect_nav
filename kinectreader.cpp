#include"kinectreader.h"
//#include "config.h"
Kinectreader::Kinectreader()
{
  //构造函数对类中的属性赋值，而不算定义变量
  result = XN_STATUS_OK; 
  //OpenCV  
  imgDepth16u=cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);  
  imgRGB8u=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
  depthShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);  
  imageShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
  
  /*B214实验室Kinect v1 相机内参，标定于2.17.4.6日，*/
  camera.cx = 321.1123;
  camera.cy = 244.5377;
  camera.fx = 519.2247;
  camera.fy = 517.6005;
  camera.depthScale = 1000.0;
  
  //相机外参赋值，标定于17.4.12日
  Tcm <<     1.0292,   0.0052,    0.0434,   -0.0384,
   -0.0052,    0.9729,    0.2207,   -0.4750,
   -0.0434,   -0.2207,    0.9621,    0.2077,
         0,         0,         0,    1.0000;
  
  
//  cloud_p = new pcl::PointCloud<pcl::PointXYZ>;
    zmin = 1.0;//Config::get<double> ("z_min");
    zmax = 2.5;//Config::get<double> ("z_max");
    voxel_grid = 0.02;//Config::get<double> ("grid_size");
    MeanK = 50;//Config::get<int> ("MeanK");
    outlier_threshold = 0.6;//Config::get<double> ("outlier_threshold");
}

void Kinectreader::Init()//初始化函数，完成reader的初始配置设置
{
  //打开两个窗口 
//  cvNamedWindow("depth",1);  
//  cvNamedWindow("image",1);
  //context 初始化，这样所有的OpenNI相关的模块都被读取分析，直到shutdown函数被调用
  result = context.Init();   
  CheckOpenNIError( result, "initialize context" );    
      
  // creategenerator           
  result = depthGenerator.Create( context );   
  CheckOpenNIError( result, "Create depth generator" );    
          
  result = imageGenerator.Create( context );   
  CheckOpenNIError( result, "Create image generator" );  
  //map mode  setting  
  mapMode.nXRes = 640;    
  mapMode.nYRes = 480;   
  mapMode.nFPS = 30;   
  result = depthGenerator.SetMapOutputMode( mapMode );   
  CheckOpenNIError( result, "Set map output mode" ); 
  result = imageGenerator.SetMapOutputMode( mapMode );    
  CheckOpenNIError( result, "set map output mode" );  
  // correct view port    
  depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator );   //设置到彩图视角下
  //streaming data  
  result = context.StartGeneratingAll(); 
  CheckOpenNIError( result, "start generating all" ); 
}

IplImage* Kinectreader::ReadKinect()//读取数据流
{
  result = context.WaitNoneUpdateAll();   //先更新数据，强制更新
  depthGenerator.GetMetaData(depthMD);   
  imageGenerator.GetMetaData(imageMD); 
    memcpy(imgDepth16u->imageData,depthMD.Data(),640*480*2);  //成员数据拷贝 每个像素点含有两个字节
    
}

void Kinectreader::ShowDepthAndImg()
{
  //memcpy(imgDepth16u->imageData,depthMD.Data(),640*480*2);  //成员数据拷贝 每个像素点含有两个字节
  cvConvertScale(imgDepth16u,depthShow,255/4096.0,0);  //转换成两个字节的灰度图显示
  memcpy(imgRGB8u->imageData,imageMD.Data(),640*480*3);  //RGB真彩图由三个字节构成
  cvCvtColor(imgRGB8u,imageShow,CV_RGB2BGR);  
  
  
  cvShowImage("depth", depthShow);  
  
  cvShowImage("image",imageShow); 
  //if(cvWaitKey())
  //cvSaveImage("depth.png", imgDepth16u);
  //cvSaveImage("rgb.png", imgRGB8u);
}

void Kinectreader::StopRead()
{
  //destroy  
  cvDestroyWindow("depth");  
  cvDestroyWindow("image");  
  cvReleaseImage(&imgDepth16u);  
  cvReleaseImage(&imgRGB8u);  
  cvReleaseImage(&depthShow);  
  cvReleaseImage(&imageShow);  
  context.StopGeneratingAll();  
  context.Release();  
 
}

void Kinectreader::CheckOpenNIError( XnStatus result, std::string status )  
{   
  if( result != XN_STATUS_OK )   
    cerr << status << " Error: " << xnGetStatusString( result ) << endl;  
}  

pcl::PointCloud<pcl::PointXYZ>::Ptr Kinectreader::ToPointCloud()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);//待转化的3D点云指针
  //pcl::visualization::CloudViewer viewer("cloud");//点云显示**此处新建viewer会有问题，程序会崩掉
  ushort *tmp = new ushort[1];
  pcl::PointXYZ p;//像素点对应的3D点
  //根据相机模型逐点进行像素点到三维空间坐标的转换
  for(int m = 0; m < imgDepth16u->height; m++)	      
  for(int n = 0; n < imgDepth16u->width; n = n+2)
  {
  *tmp = ((ushort*)(imgDepth16u->imageData + m*imgDepth16u->widthStep))[n];
  ushort d = *tmp;//= imgDepth16u->imageData(m)[n];
  if(d == 0)//对于不存在的点应舍去
  continue;
  //根据相机内参计算点云在相机坐标系下的空间位置//********这里的x,y有可能取反了。。。***********//
  p.z =  double(d) / camera.depthScale;//转换成视野里正对着的视角，z方向取反，同时注意直通滤波器的范围也要做相应的改变。
  p.x =  (n - camera.cx) * p.z / camera.fx;//x方向取反
  p.y =  (m - camera.cy) * p.z / camera.fy;
  
    //根据相机外参将相机坐标转换成机器人坐标系下
  
  
  cloud_p->points.push_back(p);    
  
  }
  cloud_p->height = 1;
  cloud_p->width = cloud_p->points.size();
  cout << "points size: " << cloud_p->points.size() << endl;
  cloud_p->is_dense = false;
  cloud_p->points.resize(cloud_p->width * cloud_p->height);
  
      //进行滤波
    //直通滤波器，去除较远处的点，设置z 方向的显示范围 
    pass.setInputCloud(cloud_p);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin, zmax);//由于前面为了正确显示点云，对p.z取反，此处对z坐标的范围设置为（-3.0, 0.0），此数值可根据需要更改。
    pass.filter(*cloud_p);
    
        
    //进行网格滤波降采样
    voxel.setLeafSize( voxel_grid, voxel_grid, voxel_grid );//设置网格滤波器的分辨率
    voxel.setInputCloud(cloud_p);
    voxel.filter( *cloud_p );//滤波后的点云存储到cloud_p中
    
    //滤波器移除离群点
    sor.setInputCloud(cloud_p);//设置输入点云
    sor.setMeanK(MeanK);//设置进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(outlier_threshold);//设置判断是否为离群点的阈值
    sor.filter(*cloud_p);//移除离群点滤波
    
    pcl::transformPointCloud(*cloud_p, *cloud_tmp, Tcm);
    //根据相机外参将相机坐标转换成机器人坐标系下
    
    
    /*
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  
  */
  return cloud_tmp;
  cout << "1" << endl;// 程序并不会执行此句
 // cloud_p->clear();
}


