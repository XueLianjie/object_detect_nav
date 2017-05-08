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


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

struct Camera
{
  double cx, cy, fx, fy, depthScale;
            
};

using namespace std;  
using namespace cv;  
      
void CheckOpenNIError( XnStatus result, string status )  
{   
  if( result != XN_STATUS_OK )   
    cerr << status << " Error: " << xnGetStatusString( result ) << endl;  
}  
      
int main( int argc, char** argv )  
    {  
      
        XnStatus result = XN_STATUS_OK;    
        xn::DepthMetaData depthMD;  
        xn::ImageMetaData imageMD;  
      
        //OpenCV  
        IplImage*  imgDepth16u=cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);  
        IplImage* imgRGB8u=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
        IplImage*  depthShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);  
        IplImage* imageShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);  
        cvNamedWindow("depth",1);  
        cvNamedWindow("image",1); 
	
	pcl::visualization::CloudViewer viewer("cloud");//点云显示
        char key=0;  
      
        //【2】  
        // context
	
        xn::Context context;   
        result = context.Init();   
        CheckOpenNIError( result, "initialize context" );    
      
        // creategenerator    
        xn::DepthGenerator depthGenerator;    
        result = depthGenerator.Create( context );   
        CheckOpenNIError( result, "Create depth generator" );    
        xn::ImageGenerator imageGenerator;  
        result = imageGenerator.Create( context );   
        CheckOpenNIError( result, "Create image generator" );  
      
        //【3】  
        //map mode    
        XnMapOutputMode mapMode;   
        mapMode.nXRes = 640;    
        mapMode.nYRes = 480;   
        mapMode.nFPS = 30;   
        result = depthGenerator.SetMapOutputMode( mapMode );    
        result = imageGenerator.SetMapOutputMode( mapMode );    
      
        //【4】  
        // correct view port    
        depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator );   //设置到彩图视角下
      
        //【5】  
        //read data  
        result = context.StartGeneratingAll();    
        
        //【6】  
        
        result = context.WaitNoneUpdateAll();    
      
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);//待转化的3D点云指针
	    pcl::PointXYZ p;//像素点对应的3D点
	    //pcl::visualization::CloudViewer viewer("cloud");//点云显示
	    
	    Camera camera;
	    camera.cx = 325.5;
	    camera.cy = 253.5;
	    camera.fx = 518.0;
	    camera.fy = 519.0;
	    camera.depthScale = 1000.0;
	    
	    ushort *tmp = new ushort[1];
        while( key!=27  )   // && !(result = context.WaitNoneUpdateAll( ))
        {    
	  result = context.WaitNoneUpdateAll( );
            //get meta data  
            depthGenerator.GetMetaData(depthMD);   
            imageGenerator.GetMetaData(imageMD);  
      
            //【7】  
            //OpenCV output  
            memcpy(imgDepth16u->imageData,depthMD.Data(),640*480*2);  //成员数据拷贝 每个像素点含有两个字节
            cvConvertScale(imgDepth16u,depthShow,255/4096.0,0);  //转换成两个字节的灰度图显示
            memcpy(imgRGB8u->imageData,imageMD.Data(),640*480*3);  //RGB真彩图由三个字节构成
            cvCvtColor(imgRGB8u,imageShow,CV_RGB2BGR);  
            cvShowImage("depth", depthShow);  
	    //ss << i+1;

	    //ss >> filename;
	    //cvSaveImage("depth.png", imgDepth16u);
	    
            cvShowImage("image",imageShow);  
	    
	    //转换成点云
	    

	    //根据相机模型逐点进行像素点到三维空间坐标的转换
	    for(int m = 0; m < imgDepth16u->height; m++)
	      
	    for(int n = 0; n < imgDepth16u->width; n = n+2)
	    {
	      *tmp = ((ushort*)(imgDepth16u->imageData + m*imgDepth16u->widthStep))[n];
	      //*tmp = ((uchar*)(imgDepth16u->imageData + m*imgDepth16u->widthStep))[n+1];
	      //*tmp = ((uchar *)(queryGrey->imageData + i*queryGrey->widthStep))[j];
	      ushort d = *tmp;//= imgDepth16u->imageData(m)[n];
	      if(d == 0)//对于不存在的点应舍去
		continue;
	      //根据相机内参计算点云在相机坐标系下的空间位置
	      p.z = - double(d) / camera.depthScale;//转换成视野里正对着的视角，z方向取反，同时注意直通滤波器的范围也要做相应的改变。
	      p.x = - (n - camera.cx) * p.z / camera.fx;//x方向取反
	      p.y = (m - camera.cy) * p.z / camera.fy;
	      cloud_p->points.push_back(p);
	      
	    }  
	    
	    cloud_p->height = 1;
	    cloud_p->width = cloud_p->points.size();
	    cout << "points size: " << cloud_p->points.size() << endl;
	    cloud_p->is_dense = false;
	    cloud_p->points.resize(cloud_p->width * cloud_p->height);
	   // cloud_p->points
	    viewer.showCloud(cloud_p);
	    cloud_p->clear();

	    
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = cloud_p;
	// Read a PCD file from disk.
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
//	{
//		return -1;
//	}

	// Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
	segmentation.segment(*planeIndices, *coefficients);

	if (planeIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(planeIndices);
		extract.filter(*plane);

		// Retrieve the convex hull.
		pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(plane);
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		// Redundant check.
		if (hull.getDimension() == 2)
		{
			// Prism object.
			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
			prism.setInputCloud(cloud);
			prism.setInputPlanarHull(convexHull);
			// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
			// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
			prism.setHeightLimits(0.0f, 0.1f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*objects);
			pcl::visualization::CloudViewer viewerObjects("Objects on table");
			viewerObjects.showCloud(objects);
			while (!viewerObjects.wasStopped())
			{
				// Do nothing but wait.
			}
		}
		else std::cout << "The chosen hull is not planar." << std::endl;
	}

	    
	    
	    //delete cloud_p;
            key=cvWaitKey(20);  //延时2ms
        }  
      
        //destroy  
        cvDestroyWindow("depth");  
        cvDestroyWindow("image");  
        cvReleaseImage(&imgDepth16u);  
        cvReleaseImage(&imgRGB8u);  
        cvReleaseImage(&depthShow);  
        cvReleaseImage(&imageShow);  
        context.StopGeneratingAll();  
        context.Release();  
        return 0;  
    }
    
  //  IplImage* CvCapture_OpenNI::retrievePointCloudMap() {     int cols = depthMetaData.XRes(), rows = depthMetaData.YRes();     if( cols <= 0 || rows <= 0 )         return 0;      cv::Mat depth;     getDepthMapFromMetaData( depthMetaData, depth, noSampleValue, shadowValue );      const float badPoint = 0;     cv::Mat pointCloud_XYZ( rows, cols, CV_32FC3, cv::Scalar::all(badPoint) );      for( int y = 0; y < rows; y++ )     {         for( int x = 0; x < cols; x++ )         {              unsigned short d = depth.at(y, x);             // Check for invalid measurements             if( d == CvCapture_OpenNI::INVALID_PIXEL_VAL ) // not valid                 continue;              XnPoint3D proj, real;             proj.X = x;             proj.Y = y;             proj.Z = d;             depthGenerator.ConvertProjectiveToRealWorld(1, &proj, &real);             pointCloud_XYZ.at(y,x) = cv::Point3f( real.X*0.001f, real.Y*0.001f, real.Z*0.001f); // from mm to meters         }     }      outputMaps[CV_CAP_OPENNI_POINT_CLOUD_MAP].mat = pointCloud_XYZ;      return outputMaps[CV_CAP_OPENNI_POINT_CLOUD_MAP].getIplImagePtr(); }
    
