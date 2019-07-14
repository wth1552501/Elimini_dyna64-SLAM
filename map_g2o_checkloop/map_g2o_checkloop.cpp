#include <iostream>
#include <fstream>
#include <boost/timer.hpp>
#include <boost/format.hpp>  // for formating strings
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h>//终端控制定义
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <boost/format.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

//g2o
#include "myslam/g2o_types.h"
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>

#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings

//openni
#include "OpenNI.h"

//user define
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/zx.h"

int endx;
int endz;

//定义点云的类型

//在这里提前定义一下相机的参数，就不用载入的办法了

double cx=316.2095;
double cy=263.7301;
double fx=514.6804;
double fy=514.8313;
double depthScale=5000;
double ch;
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

vector<SE3> vector_transform_matrix;
vector<cv::Mat> vector_rvec;
vector<cv::Mat> vector_tvec;
vector<Eigen::Matrix <double,4,4>> vector_M_for_track;
vector<cv::Affine3f> vector_for_pose;
//点云显示
boost::mutex updateModelMutex;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
        viewer->addPointCloud<PointT> (cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->initCameraParameters ();
        return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
        while (!viewer->wasStopped ())
        {
               viewer->spinOnce (100);
               boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }
}

int check_loop(int i)
{
  return 1;
}
  void  viz_pose()
  {
    
  }
  void  viz_track()
  {
    
      // visualization
      cv::viz::Viz3d vis_track("Visual Odometry");
      cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
      vis_track.setBackgroundColor(cv::viz::Color::black());
  
      // draw the trace
      cv::Point3f point_begin(0.0, 0.0, 0.0);
      cv::Point3f point_end;
  
      //cv::viz::WLine wline(cv::Point3f(0, 0, 0), (100, 100, 100), cv::Scalar(0, 0, 255));
      cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
      cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
      vis_track.setViewerPose(cam_pose);
  
      world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
      camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
      vis_track.showWidget("World", world_coor);
      vis_track.showWidget("Camera", camera_coor);
      ///////////////////////initialization of vis_track//////////////////////////////////////
  
       Eigen::Matrix <double,4,4> T;
       cv::Affine3f M;
          //*******************************************************************************************
          // 画出轨迹
          //*******************************************************************************************
       vector<cv::viz::WLine> lines;
       cout<<"vector_M_for_track.size()"<<vector_M_for_track.size()<<endl;
       cout<<"vector_for_pose.size()"<<vector_for_pose.size()<<endl;
       int i(0);
      for(int j(1);j<=vector_M_for_track.size()-1;j++)
      {
	  T=vector_M_for_track[j];
	  M=vector_for_pose[j];
          point_end = cv::Point3f( T(0,3),T(1,3),T(2,3));
	  //point_end = cv::Point3f( 10.0f,10.0f,10.0f);
	  cout<<"1"<<endl;
          cv::viz::WLine line(point_begin, point_end, cv::viz::Color::green());
	  lines.push_back(line);
	  cout<<"2"<<endl;
          //vis_track.spinOnce(2000, false);
	  //showWidget的用法看起来是用以点特殊的，这里第一次使用进行了报错，显示我是在误用。
          point_begin = point_end; 
          //vis_track.setWidgetPose("Camera", M);
      }
      cout<<"lines.size() is"<<lines.size()<<endl;
      for (vector<cv::viz::WLine>::iterator iter = lines.begin(); iter != lines.end(); iter++)
         {
             string id = to_string(i);
             vis_track.showWidget(id, *iter);
	     vis_track.spinOnce(20000, false);
             i++;
         }
      vis_track.spinOnce(20000, false);
      vis_track.saveScreenshot("KeyFrameTrajectory.png");
 }
 
int compare_color(cv::Vec3b a_color,cv::Vec3b b_color)
{
  //cout 完整的Vec3b值可以，但是单独输出一位，就会出错的。
  
  if((fabs(a_color[0]-b_color[0])<=5)&&(fabs(a_color[1]-b_color[1])<=5)&&(fabs(a_color[2]-b_color[2])<=5))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int main ( int argc, char** argv )
{
/**************OPENNI接口读取Kinect*******************/
	//打开串口

/**************************************************/
/**************视觉里程计配置************************/

    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    //定义参数文件,相机内参等参数通过Config读取
    myslam::Config::setParameterFile ( argv[1] );
    //定义相机
    myslam::Camera::Ptr camera ( new myslam::Camera );
    // 视觉里程计
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
	

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );
	
    //点云操作
    // 计算点云并拼接

    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(pointCloud);


/*********************开始地图搭建**************************/
    //计时器
    double Time = (double)cvGetTickCount();
    double loop_Time = (double)cvGetTickCount();
/**************************************************/
    double flag(1);
    //开始采集图像q
    
    cv::Vec3b color_temp;
	      color_temp[0]=255;
	      color_temp[1]=255;
	      color_temp[2]=255;
	      
    while(flag<=8)
    {	
      
       boost::timer loop_timer;
		//运行时间
        double Time_Running =(double)cvGetTickCount() - loop_Time;
		
        
        cv::Mat mScaledDepth;      
        boost::format fmt( "%s%d.%s" );
        const cv::Mat cImageBGR=cv::imread((fmt%"/home/wth/design/Myslam/dataset/bgr/bgr"%(flag)%"jpg").str());
	const cv::Mat converted_cImageBGR=cv::imread((fmt%"/home/wth/design/Myslam/dataset/bgr/converted_bgr"%(flag)%"jpg").str());
	//cv::cvtColor( newmImageBGR, cImageBGR, CV_RGB2BGR );
	//cImageBGR is used now
        cv::Mat mImageDepth=cv::imread((fmt%"/home/wth/design/Myslam/dataset/depth/"%(flag)%"png").str(),-1);
	mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / 1000 );
	  
        
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = cImageBGR;
        pFrame->depth_ = mImageDepth;
        pFrame->time_stamp_ = Time_Running;
		// 相机内参
		double cx = camera->cx_;
		double cy = camera->cy_;
		double fx = camera->fx_;
		double fy = camera->fy_;
		double depthScale = camera->depth_scale_;
		
		//定时
		boost::timer timer;
		
        //加入视觉里程计
        vo->addFrame ( pFrame );
		
		//如果位姿丢失
        if(vo->state_ == vo->LOST)
        {
        	cout<<"VO has lost."<<endl;
        	break;
        }
		//输出位姿估计耗时
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        //变换矩阵求逆
        SE3 Twc = pFrame->T_c_w_.inverse();
	Eigen::Matrix <double,4,4> new_temp;
	new_temp=Twc.matrix();
	vector_M_for_track.push_back(new_temp);
	////////////////////////////////////////////////
	//Modified by Tianhao Wang on 5.31
	//this is a global vector
	vector_transform_matrix.push_back(Twc);
	//this vector is used to storage M for drawing the track 
	////////////////////////////////////////////////
	vector_tvec.push_back(pFrame->frame_tvec);
	vector_rvec.push_back(pFrame->frame_rvec);
        // 显示图像和相机位姿
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );
	vector_for_pose.push_back(M);
        //位姿显示
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
		
        // 显示出深度图像
        cv::imshow( "Depth Image", mScaledDepth );

        //显示特征图
        Mat img_show = cImageBGR.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "Image Mappoint", img_show );
        cv::waitKey ( 1 );
        //绘制点云
        cv::Mat colorPCL = cImageBGR.clone(); 
        cv::Mat depthPCL = mImageDepth.clone();
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//我现在深度怀疑BGR图和DEPTH图是不是对齐的，如果不是对齐的可怎么办
				
        for ( int v=0; v<colorPCL.rows; v++ )
        {
            for ( int u=0; u<colorPCL.cols; u++ )
            {
	      
	        if(compare_color(colorPCL.at<cv::Vec3b>(v,u),color_temp)==1)
		{
		  continue;
		}
                unsigned int d = depthPCL.ptr<unsigned short> (v)[u]; 
		// 深度值
                if ( d==0 ) continue; // 为 0 表示没有测量到
		if( d>120000)continue;//深度太大不稳定忽略
				
                Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Vector3d pointnow=pFrame->camera_->camera2world(point,pFrame->T_c_w_);
                PointT p ;
                p.x = pointnow[0];
                p.y = pointnow[1];
                p.z = pointnow[2];
                p.b = colorPCL.data[ v*colorPCL.step+u*colorPCL.channels() ];
                p.g = colorPCL.data[ v*colorPCL.step+u*colorPCL.channels()+1 ];
                p.r = colorPCL.data[ v*colorPCL.step+u*colorPCL.channels()+2 ];
		
		pointCloud->push_back( p );	
				
            }
        }
		
	  //实时显示//
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> color (pointCloud);
        viewer->updatePointCloud<pcl::PointXYZRGB>(pointCloud,color,"sample cloud");
        viewer->spinOnce(10);
	  
	  // 终止图像采集快捷键
	char key=cv::waitKey(1);
        if( key == 'j')
        {
	  //pcl::io::savePCDFile<pcl::PointXYZRGB>("newpcl.pcd",*pointCloud);
	  break;
	}
	if(flag==8)
	{
	  //pointCloud->is_dense = false;
	  pcl::io::savePCDFile<pcl::PointXYZRGB>("newpcl_just_map.pcd",*pointCloud);
	  std::cout<<"************"<<endl<<"************"<<endl;
	  std::cout<<"task is over"<<std::endl;
	  std::cout<<"************"<<endl<<"************"<<endl;
	  break;
	}
	
	cout<<"Loop costs time: "<<loop_timer.elapsed() <<endl;
	flag++;
    }
    
     /////////////////////////////////////////////////////////////////////////////////////////////
     //filter part
     //destroy the noise signal
      // 读取PCD文件
     pcl::io::savePCDFile<pcl::PointXYZRGB>("newpcl_just_map.pcd",*pointCloud);
     pcl::PCDReader reader;
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
     reader.read ("newpcl_just_map.pcd", *cloud_blob);
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZRGB>);//
     pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid; 
     voxelgrid.setInputCloud(cloud_blob);
     //输入点云数据
     voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);
     //AABB长宽高
     voxelgrid.filter(*cloud_after_voxelgrid);
     pcl::io::savePCDFile<pcl::PointXYZRGB>("newpcl_after_filter.pcd",*cloud_after_voxelgrid);
     /////////////////////////////////////////////////////////////////////////////////////////////
/*********************显示姿态**************************/
/*********************显示轨迹**************************/
     //viz_track();


     //Modified by Tianhao Wang on 5.19, G20 and Check Loop module
          // 选择优化方法
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      

      //typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
     //  typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
      // 初始化求解器
      //SlamLinearSolver* linearSolver = new SlamLinearSolver();
      //linearSolver->setBlockOrdering( false );
      //SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
      //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
      //g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
      //globalOptimizer.setAlgorithm( solver ); 
   
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
      Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
      Block* solver_ptr = new Block ( linearSolver );
      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
      g2o::SparseOptimizer globalOptimizer;
      globalOptimizer.setAlgorithm ( solver );
      // 不要输出调试信息
      //globalOptimizer.setVerbose( false );
      //////////////////////////////////////////////////////////////
      //Add robust kernel to avoid negative influence caused by error check_loop
      static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
      //
      /////////////////////////////////////////////////////////////////////////////////////////////////////
      // 向globalOptimizer增加第一个顶点
      g2o::VertexSE3* v = new g2o::VertexSE3();
      v->setId(0);
      v->setEstimate( Eigen::Isometry3d::Identity()); 
      //估计为单位矩阵
      v->setFixed( true ); 
      //第一个顶点固定，不用优化
      globalOptimizer.addVertex( v );
      //Attention vecotr[1] is the first element 
      //because the first frame has not a correspnding transform matrix
    
      for (int i_twc_temp(1);i_twc_temp<=vector_rvec.size()-1;i_twc_temp++)
      {
         // cloud = joinPointCloud( cloud, currFrame, T, camera );
         // 向g2o中增加这个顶点与上一帧联系的边
         // 顶点部分
         // 顶点只需设定id即可
         g2o::VertexSE3 *v = new g2o::VertexSE3();
         v->setId(i_twc_temp);
         v->setEstimate( Eigen::Isometry3d::Identity() );
         globalOptimizer.addVertex(v);
         // 边部分
         g2o::EdgeSE3* edge = new g2o::EdgeSE3();
         // 连接此边的两个顶点id
         edge->vertices() [0] = globalOptimizer.vertex(i_twc_temp-1);
         edge->vertices() [1] = globalOptimizer.vertex(i_twc_temp);
	 ///

	 //this place should add two id,deciding which two vertices links to edges
         // 信息矩阵
         Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
         // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
         // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
         // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
         information(0,0) = information(1,1) = information(2,2) = 100;
         information(3,3) = information(4,4) = information(5,5) = 100;
         // 也可以将角度设大一些，表示对角度的估计更加准确
         edge->setInformation( information );
         // 边的估计即是pnp求解之结果
	 SE3 Twc=vector_transform_matrix[i_twc_temp];

      
	Eigen::Matrix <double,4,4> NEW;
	NEW=Twc.matrix();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for(int i(0);i<=3;i++)
	{
	  for(int j(0);j<=3;j++)
	  {
	    T(i,j) =NEW(i,j) ; 
	  }
	}
	
        //edge->setMeasurement( T.inverse() );
	edge->setMeasurement( T);
	 
	
         // 将此边加入图中
         globalOptimizer.addEdge(edge);
	 cout<<"the "<<i_twc_temp<<" th adding edge"<<endl;

     }
	cout<<"first step for g2o is over,the next step is loop"<<endl; 
	
	
	/*
	int i_loop(2);	 
	 //loop for adding more valid edge
	 while(i_loop<vector_transform_matrix.size())
	 //control i and j;
	 {
	  if(check_loop(i_loop))
	  {
	      g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	      // 连接此边的两个顶点id
	      edge->vertices() [0] = globalOptimizer.vertex(i_loop-2);
	      edge->vertices() [1] = globalOptimizer.vertex(i_loop);
	      edge->setRobustKernel( robustKernel );
	      // 信息矩阵
	      Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
	      // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
	      // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
	      // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
	      information(0,0) = information(1,1) = information(2,2) = 100;
	      information(3,3) = information(4,4) = information(5,5) = 100;
	      // 也可以将角度设大一些，表示对角度的估计更加准确
	      edge->setInformation( information );
	      // 边的估计即是pnp求解之结果
	      // 边的估计即是pnp求解之结果
	      SE3 Twc=vector_transform_matrix[i_loop-1];
	      Mat tvec_temp=vector_tvec[i_loop-1];
	      Mat rvec_temp=vector_rvec[i_loop-1];
	      cv::Mat R;
	      cv::Rodrigues( rvec_temp, R );
	      Eigen::Matrix3d r;
	      cv::cv2eigen(R, r);
	      // 将平移向量和旋转矩阵转换成变换矩阵
	      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	      Eigen::AngleAxisd angle(r);
	      Eigen::Translation<double,3> trans(tvec_temp.at<double>(0,0), tvec_temp.at<double>(0,1), tvec_temp.at<double>(0,2));
	      T = angle;
	      T(0,3) = tvec_temp.at<double>(0,0); 
	      T(1,3) = tvec_temp.at<double>(0,1); 
	      T(2,3) = tvec_temp.at<double>(0,2);
	      
	      
	      
	      Twc=vector_transform_matrix[i_loop];
	      tvec_temp=vector_tvec[i_loop];
	      rvec_temp=vector_rvec[i_loop];
	      cv::Mat R_new;
	      cv::Rodrigues( rvec_temp, R_new);
	      Eigen::Matrix3d r_new;
	      cv::cv2eigen(R_new, r_new);
	      // 将平移向量和旋转矩阵转换成变换矩阵
	      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	      Eigen::AngleAxisd angle_new(r_new);
	      Eigen::Translation<double,3> trans_new(tvec_temp.at<double>(0,0), tvec_temp.at<double>(0,1), tvec_temp.at<double>(0,2));
	      t = angle_new;
	      t(0,3) = tvec_temp.at<double>(0,0); 
	      t(1,3) = tvec_temp.at<double>(0,1); 
	      t(2,3) = tvec_temp.at<double>(0,2);
	      
	      Eigen::Isometry3d T_final=T*t;
	      
	      edge->setMeasurement(T_final.inverse());
	      // 将此边加入图中
	      globalOptimizer.addEdge(edge);
	  }
	  i_loop++;
	 }
	 */
     // 优化所有边
     cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
     globalOptimizer.save("/home/wth/design/Myslam/Elimi_dyna64-SLAM/bin/result_before.g2o");
     globalOptimizer.initializeOptimization();
     globalOptimizer.optimize( 100 ); 
     //可以指定优化步数
     globalOptimizer.save( "/home/wth/design/Myslam/Elimi_dyna64-SLAM/bin/result_after.g2o" );
     cout<<"bothok"<<endl;
   
     cout<<"Optimization done."<<endl;
     globalOptimizer.clear();
    return 0;
}



