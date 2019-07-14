#include <iostream>
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

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>//octomap
#include <octomap/ColorOcTree.h>

//pcl
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

//openni
#include "OpenNI.h"

//user define
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/zx.h"



#define DEVICE "/dev/ttyUSB0" 

#define S_TIMEOUT 1

#define IMG_NUM 765


int serial_fd = 0; 
  
unsigned int total_send = 0 ;

/***********串口函数*************/
int angle(double destinationx,double destinationy,double startx,double starty,double directx,double directy,char*p);

int init_serial();

int uart_send(int fd, char *data, int datalen);

int uart(double destinationx,double destinationy,double startx,double starty,double directx,double directy);

//鼠标点击d回调函数
void on_Mouse(int event, int x, int y, int flags, void *param);

//终点坐标
int start_flag =0;
int endx=0;
int endz=0;

typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

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



int main ( int argc, char** argv )
{
/**************OPENNI接口读取Kinect*******************/
	//打开串口
    init_serial();
	
    //OPENNI读取Kinect相机
    openni::Status rc = openni::STATUS_OK;
    openni::Device devAnyDevice;
    openni::VideoStream streamDepth, streamColor;
    const char* deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = devAnyDevice.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("Kinect: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }
    //打开深度图像采集
    rc = streamDepth.create(devAnyDevice, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = streamDepth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("Kinect: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            streamDepth.destroy();
        }
    }
    else
    {
        printf("Kinect: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    //打开RGB图像采集
    rc = streamColor.create(devAnyDevice, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = streamColor.start();
        if (rc != openni::STATUS_OK)
        {
            printf("Kinect: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            streamColor.destroy();
        }
    }
    else
    {
        printf("Kinect: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    //检验图像采集设备接口
    if (!streamDepth.isValid() || !streamColor.isValid())
    {
        printf("Kinect: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }

    // 设置深度图像视频模式
    openni::VideoMode mModeDepth;
    // 分辨率大小
    mModeDepth.setResolution( 640, 480 );
    // 每秒10帧
    mModeDepth.setFps( 5 );
    // 像素格式
    mModeDepth.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );

    streamDepth.setVideoMode( mModeDepth);
    
    // 同样的设置彩色图像视频模式
    openni::VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 5 );
    mModeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );

    streamColor.setVideoMode( mModeColor);

    // 图像模式注册
    if( devAnyDevice.isImageRegistrationModeSupported(
        openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }
        
    // 打开深度和图像数据流1
    streamDepth.start();
    streamColor.start();

    // 创建OpenCV图像窗口
    //cv::namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
    //cv::namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );

    // 获得最大深度值
    int iMaxDepth = streamDepth.getMaxPixelValue();

    std::cout <<"the max depth:"<<iMaxDepth<<endl;


    // 循环读取数据流信息并保存在VideoFrameRef中
    openni::VideoFrameRef  frameDepth;
    openni::VideoFrameRef  frameColor;

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


	
    // 建立八叉树
    octomap::ColorOcTree tree( 0.04 );
	
    octomap::OcTree displaytree( 0.008);
/*********************开始地图搭建**************************/
    //计时器
    double Time = (double)cvGetTickCount();
    double loop_Time = (double)cvGetTickCount();
/**************************************************/
    //开始采集图像q
    while( true )
    {	boost::timer loop_timer;
		//运行时间
        double Time_Running =(double)cvGetTickCount() - loop_Time;
		
        // 读取图像数据流
        streamDepth.readFrame( &frameDepth );
        streamColor.readFrame( &frameColor );

        // 将彩色图像数据转化成OpenCV格式
        const cv::Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
		// 首先将RGB格式转换为BGR格式
        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );

        // 将深度数据转换成OpenCV格式
        const cv::Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
        // 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );

        //没有采集到图像
        if( mImageRGB.data==nullptr || mImageDepth.data==nullptr )
		{
		  break;
		}
           
        //创建图像帧
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
		
				
        for ( int v=0; v<colorPCL.rows; v++ )
        {
            for ( int u=0; u<depthPCL.cols; u++ )
            {
                unsigned int d = depthPCL.ptr<unsigned short> ( v )[u]; // 深度值

                if ( d==0 ) continue; // 为 0 表示没有测量到
                
		if( d>8500 )continue;//深度太大不稳定忽略
				
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
		octomap::point3d newquery( p.x,p.y,p.z);
		if((displaytree.search(newquery))==NULL)
		{
		  pointCloud->points.push_back( p );
		}
				displaytree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
                tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
	            tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );

				
            }
        }
		
	  //实时显示
	  pcl::visualization::PointCloudColorHandlerRGBField<PointT> color (pointCloud);
      viewer->updatePointCloud<pcl::PointXYZRGB>(pointCloud,color,"sample cloud");
      viewer->spinOnce(10);
	  
	  //八叉树投影
	  octomap::OcTreeNode* result;
	  int tu[500][500];  
	  IplImage *img = cvCreateImage( cvSize(500,500),IPL_DEPTH_8U,3);
	  for(int x=0;x<500;x++)
	  {
		for(int y=0;y<500;y++)
		{
		  cvSet2D(img,x,y,cvScalar(255,255,255));
		}
	  }
	  
	  for(int y=-50; y<0; y+=4)
	  {
		for(int x=-250; x<250; x+=2)
		  {
			for(int z=-250; z<250; z+=2)
			{
				
				octomap::point3d newquery((float) x*0.01f,(float) y*0.01f,(float) z*0.01f);
				result=tree.search(newquery); 
				if (result!=NULL)
				{
				  tu[x+250][z+250]=1;
				  if(tu[x+250][z+250]==1)
				  {
					for(int i=-2;i<2;i++)
					{
					  for(int j=-2;j<2;j++)
					  {
						
						if((x+250+i)>=500 || (x+250+i)<0 || (z+250+j)>=500 || (z+250+j)<0)
						{
						  continue;
						}
						cvSet2D(img,x+250+i,z+250+j,cvScalar(255,0,0));
					  }
					}
				  }
				}
			}
		  }
	  }
		
	  //二维地图位移
	  double startx =100* Twc.translation()( 0,0) +250;
	  double startz =100* Twc.translation()( 2,0) +250;
	  Eigen::Vector3d direction(0,0,50);
	  Eigen::Vector3d rotation=Twc.rotation_matrix()*direction;
	  
	  double rotation_x=rotation(0,0)+startx;
	  double rotation_z=rotation(2,0)+startz;
	  
	  std::cout<<"current point:"<<startx<<" "<<startz<<std::endl;
	  cout<<"vector route start:"<<m_ResultList.size()<<endl;
	  if(start_flag ==1)
	  {
		Arout(img,round((double)startx/10),round((double)startz/10));
	  }
	  cout<<"vector route end:"<<m_ResultList.size()<<endl;
	 
	  Mat img_print=cv::cvarrToMat(img);
	  
	  int route_x = 0;
	  int route_z = 0;
	  
	  double direction_x=0;
	  double direction_z=0;
	  node* route_point=m_ResultList.front();
	  int route_count = 0;
	  
	  int run_flag = 1;
	  if(m_ResultList.empty())
	  {
		 run_flag = 0;
	  }
	  while(!m_ResultList.empty())
	  {
		node* route_point=m_ResultList.back();
		route_x = route_point->x;
		route_z = route_point->y;
		if( route_count < 4)
		{
		  direction_x = direction_x + route_point->x;
		  direction_z = direction_z + route_point->y;
		  route_count ++;
		}
		cv::circle ( img_print, cv::Point2d( route_z*10,route_x*10 ), 2, cv::Scalar ( 0,0,255 ), -1 );
		m_ResultList.pop_back();
	  }
	  if(route_count != 0)
	  {
		direction_x = direction_x/route_count;
		direction_z = direction_z/route_count;
	  }else
	  {
		direction_x = startx/10;
		direction_z = startz/10;
	  }
	  //起点路线
	  cv::circle ( img_print, cv::Point2d( startz,startx ), 3, cv::Scalar ( 0,255,255 ), 2 );
	  cv::circle ( img_print, cv::Point2d( endz*10,endx*10 ), 3, cv::Scalar ( 255,255,0 ), 2 );
 	  cv::line(img_print, cv::Point2d( startz,startx), cv::Point2d( direction_z*10,direction_x*10 ), cv::Scalar ( 255,0,0 ),2);
 	  cv::line(img_print, cv::Point2d( startz,startx ), cv::Point2d( rotation_z,rotation_x), cv::Scalar ( 0,0,255 ),2);

	  //目标方向与实际转向
	  double direct[2]={10*direction_x-startx,10*direction_z-startz};
	  double orient[2]={rotation(0,0),rotation(2,0)};
	  double orientT[2]={rotation(2,0),-rotation(0,0)};
	  double ab,ab_d,a1,b1,cosr;
	  ab=direct[0]*orient[0]+direct[1]*orient[1];
	  ab_d=direct[0]*orientT[0]+direct[1]*orientT[1];
	  a1=sqrt(direct[0]*direct[0]+direct[1]*direct[1]);
	  b1=sqrt(orient[0]*orient[0]+orient[1]*orient[1]);
	  cosr=ab/(a1*b1);

	  if(run_flag ==1)
	  {
		if(cosr > 0.9 && cosr <=1)
		{
		  char byte[2];
		  byte[0] = 'w';
		  byte[1]	= ';';
		  cout<<"转向: 前进 夹角: "<<cosr<<endl;
		  uart_send(serial_fd,byte,2);
		}else
		{
		  char byte[2];
		  if(ab_d > 0)
		  {
			byte[0] = 'd';
			byte[1] = ';';
			cout<<"转向: 右 夹角: "<<cosr<<endl;
			uart_send(serial_fd,byte,2);
		  }else
		  {
			byte[0] = 'a';
			byte[1] = ';';
			cout<<"转向: 左 夹角: "<<cosr<<endl;
			uart_send(serial_fd,byte,2);
		  }
		}	  
	  }else
	  {
		  char byte[2];
		  byte[0] = 'x';
		  byte[1]	= ';';
		  cout<<"转向: 停止"<<endl;
		  uart_send(serial_fd,byte,2);
	  }
	  m_ResultList.clear();
	  cvNamedWindow("2-demension plot",CV_WINDOW_AUTOSIZE);
	  cv::imshow("2-demension plot",img_print);
	  cvSetMouseCallback("2-demension plot",on_Mouse,&img);

	  // 终止图像采集快捷键
	  char key=cv::waitKey(1);
		  if( key == 'q')
			  break;
	  cout<<"Loop costs time: "<<loop_timer.elapsed() <<endl;
    }
/*********************结束地图搭建**************************/

    // 关闭数据流
    streamDepth.destroy();
    streamColor.destroy();

    // 关闭设备q
    devAnyDevice.close();

    // 最后关闭OpenNI
    openni::OpenNI::shutdown();
	char byte[2];
	byte[0] = 'x';
	byte[1]	= ';';
	cout<<"转向: 停止"<<endl;
	uart_send(serial_fd,byte,2);
    //更新八叉树q
    tree.updateInnerOccupancy();
    cout<<"saving octomap ... "<<endl;
    //tree.writeBinary("octomap.bt" );
	tree.write("octomap.ot" );

    return 0;
}


//串口配置
int angle(double destinationx,double destinationy,double startx,double starty,double directx,double directy,char*p)
{
  int a[2]={0};
  int b[2]={0};
  char order[2];
  a[0]=directx;
  a[1]=directy;  
  b[0]=destinationx-startx;
  b[1]=destinationy-starty;
  double a1=sqrt(a[0]*a[0]+a[1]*a[1]);
  double b1=sqrt(b[0]*b[0]+b[1]*b[1]);
  double outangle=(a[0]*b[0]+a[1]*b[1])/(a1*b1);
  
  p=&order[0];
  
  if(0<outangle<0.5)
  order[1]='1';
  else if(0.5<=outangle<0.86)
  order[1]='2';
  else if(0.86<=outangle)
  order[1]='3';
  
  if(destinationy>starty)
    order[0]='L';
  else if(destinationy=starty)
    order[0]='F';
  else if(destinationy<starty)
    order[0]='R';
}             //directxy and startxy


//打开串口并初始化设置 

int init_serial() 
{ 
    serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (serial_fd < 0) { 
        perror("open"); 
        return -1; 
    } 
      
    //串口主要设置结构体termios <termios.h> 
    struct termios options; 
      
    /**1. tcgetattr函数用于获取与终端相关的参数。 
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中 
    */ 
    tcgetattr(serial_fd, &options); 
    /**2. 修改所获得的参数*/ 
    options.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能 
    options.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位 
    options.c_cflag &= ~CRTSCTS;//无硬件流控 
    options.c_cflag |= CS8;//8位数据长度 
    options.c_cflag &= ~CSTOPB;//1位停止位
    options.c_iflag |= IGNPAR;//无奇偶检验位 
    options.c_oflag = 0; //输出模式 
    options.c_lflag = 0; //不激活终端模式 
    cfsetospeed(&options, B9600);//设置波特率 
      
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/ 
    tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读 
    tcsetattr(serial_fd, TCSANOW, &options); 
      
    return 0; 
} 
  
/** 
*串口发送数据 
*@fd:串口描述符 
*@data:待发送数据 
*@datalen:数据长度 
*/ 


int uart_send(int fd, char *data, int datalen) 
{ 
    int len = 0; 
    len = write(fd, data, datalen);//实际写入的长度 
    if(len == datalen) { 
    total_send += len ;
        printf("total_send is %d\n",total_send); 
        return len; 
    } else { 
        tcflush(fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送 
        return -1; 
    } 
    return 0; 
} 
  

int uart(double destinationx,double destinationy,double startx,double starty,double directx,double directy) 
{ 
    init_serial(); 
  
//     char buf[]="hello world"; 
//     char buf1[11] ; 
//     memset(buf1,0,sizeof(char)*11); 
    char *p;
    angle(destinationx,destinationy,startx,starty,directx,directy,p);
    char buf[]="**";
    char buf1[2];
    memset(buf1,0,sizeof(char)*2);
    while(1)
    {
        uart_send(serial_fd, buf, 2); 
        printf("\n"); 
        sleep(1);
        memset(buf1,0,sizeof(char)*2); 
    }

    close(serial_fd); 
    return 0; 
}

void on_Mouse(int event, int x, int y, int flags, void *param)
{
  
	if(event==CV_EVENT_LBUTTONDOWN)
    {
	  start_flag = 1;
      std::cout<<"click to decide where to go"<<endl;
      std::cout<<"x coordinate is"<<"  "<<x<<endl<<"y coordinate is"<<"  "<<y<<endl;
      //坐标系不同
      endx=y/10;
	  endz=x/10;
    }
}
