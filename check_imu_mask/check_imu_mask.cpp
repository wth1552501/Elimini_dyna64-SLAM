/*Modified by Tianhao Wang at 2019.3.27
 *The dataset used here is generated on my own. Every depth is correspond with bgr, as a result we needn't create a asscociate.txt.
 *This project is used to implement tracking and matching feature points based on LK-pyramid and FAST
 *Further we use the feature points to calculate fundamental matrix and epilar line.
 *Finally we judge the dynamic points. 
 */

#include <stdio.h>      
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>    
#include <errno.h>      
#include <string.h>

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <boost/timer.hpp>
#include <boost/format.hpp>  // for formating strings
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings

//openni
#include "OpenNI.h"

//g2o
/*
#include <g2o/types/slam3d/types_slam3d.h> 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
*/

//user define
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/integration.h"

#define  TRUE 0
#define  FALSE -1
#define CAMERA_IMMOBILE 0
#define CAMERA_MOBILE 1
using namespace std; 
using namespace cv;
/*Modified by Tianhao Wang at 4.20
 * To show pose of camera by using opencv viz model and IMU
 */
//
#define DEVICE "/dev/ttyACM0"
#define S_TIMEOUT 1
#define IMG_NUM 765
int serial_fd = 0;
unsigned int total_send = 0 ;
//打开串口并初始化设置
//here is an ending flag character'*'
int endx;
int endz;

//global flag for picture
vector<int> immobile_pic_flag;
vector<SE3> transform_matrix;
double cx=316.2095;
double cy=263.7301;
double fx=514.6804;
double fy=514.8313;
double depthScale=5000;
double ch;
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;


vector<cv::Vec3f> color_all;
vector<int> id_all;
vector<string> line_temp;
void read_log(string file_path)
{
        ifstream in(file_path);
	string filename;
	string line;
        string line_file;
	if(in) 
	{
		while (getline (in, line_file)) 
		  // line中不包括每行的换行符
		{ 
		  cout<<line_file<<endl;
		  line_temp.push_back(line_file);
		}
	}
	else 
	{
		cout <<"no such file" << endl;
	}
	int j(0);
	cout<<"有几行"<<line_temp.size()<<endl;
	
	   cv::Vec3f temp;
		    double b_0(0);
		    double g_1(0);
		    double r_2(0);
		    char c_b[30];
		    char c_g[30];
		    char c_r[30];
		    char c_id[30];
		    int c_flag(0);
		    int id;
	while(j<=line_temp.size()-1)
	{
               	    int i(1);
		    line=line_temp[j];
	            cout<<"whether line is not nan"<<line<<endl;
		    cout << line_temp[j]<< endl;
		    //extract and get a vec3b element
		    c_flag=0;
		    while(line[i]!=',')
		    {
		      c_b[c_flag]=line[i];
		      c_flag++;
		      i++;
		    }
		    cout<<"c_b"<<c_b<<endl;
		    cout<<1<<endl;
		    i=i+2;
		    c_flag=0;
		    while(line[i]!=',')
		    {
		      c_g[c_flag]=line[i];
		      c_flag++;
		      i++;
		    }
		    i=i+2;
		    c_flag=0;
		    cout<<"c_g"<<c_g<<endl;
		    cout<<2<<endl;
		    while(line[i]!=')')
		    {
		      c_r[c_flag]=line[i];
		      c_flag++;
		      i++;
		    }
		    i++;
		    c_flag=0;
		    cout<<"c_r"<<c_r<<endl;
		    cout<<3<<endl;
		    while(i<=line.size()-1)
		    {
		    c_id[c_flag]=line[i];
		    c_flag++;
		    i++;
		    }
		    cout<<4<<endl;
		    b_0=atof(c_b);
		    cout<<"b_0"<<b_0<<endl;
		    g_1=atof(c_g);
		    cout<<"g_1"<<g_1<<endl;
		    r_2=atof(c_r);
		    cout<<"r_2"<<r_2<<endl;
		    id=atoi(c_id);
		    cout<<"id"<<id<<endl;
		    temp[0]=b_0;
		    temp[1]=g_1;
		    temp[2]=r_2;
		    color_all.push_back(temp);
		    id_all.push_back(id);
		    j++;
		    
		    //Added by Tianhao Wang
		    memset(c_b,'\0',sizeof(c_b));
		    memset(c_g,'\0',sizeof(c_g));
		    memset(c_r,'\0',sizeof(c_r));
		    memset(c_id,'\0',sizeof(c_id));
	 	    //
	}	  
}
//Modified by Tianhao Wang
//input file_name
//output in global array color_all and id_all
//the ith element delegates that its number is id_all[i] and its color is color_all[i]

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

int init_serial()
{
    serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        std::perror("open");
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
    cfsetospeed(&options, B115200);//设置波特率

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读
    tcsetattr(serial_fd, TCSANOW, &options);

    return 0;
}

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

void imu_serial_read()
{
  init_serial();
  //设置接收缓冲区的大小
  char buff[1024];
  char end_flag[1];
  end_flag[0]='*';

  //可以一个一个字节的接收 
  char uart_0=0;
  char buff_pitch[6];
  char buff_yaw[6];
  char buff_roll[6];
  char buff_acc_x[6];
  char buff_acc_y[6];
  char buff_acc_z[6];
  int nread;
  double sta_pitch(0);
  double previous_pitch(0);
  double sta_yaw(0);
  double previous_yaw(0);
  double sta_roll(0);
  double previous_roll(0);
  double sta_acc_x(0);
  double previous_acc_x(0);
  double sta_acc_y(0);
  double previous_acc_y(0);
  int initial_flag(0);
  double ini_pitch(0);
  double ini_yaw(0);
  double ini_roll(0);
  
  viz::Viz3d window("IMU window"); 
  window.showWidget("Coordinate", viz::WCoordinateSystem()); 
  //创建平面 
  viz::WPlane plane; 
  //添加平面，并设置一个ID为plane 
  window.showWidget("plane", plane); 
  
  
  //创建一个1*3的rotation vector 
  Mat rvec = Mat::zeros(1, 3, CV_32F); 
  //Modified by Tianhao Wang on May,8th
  //new visualization
    // visualization
    cv::viz::Viz3d vis_IMU ( "IMU" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis_IMU.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis_IMU.showWidget ( "World", world_coor );
    vis_IMU.showWidget ( "IMU", camera_coor );
	
    myslam::IMU_integration imu1;
    myslam::IMU_integration imu2;
  //
  while(!vis_IMU.wasStopped())
  {
    //如果串口缓冲区中有数据 
    if((nread = read(serial_fd,buff,1024))>0)
    {
      //strcat(buff,end_flag);
      int longth=strlen(buff);
      cout<<"长度是"<<longth<<endl;
      /*for(int j(0);j<=longth-1;j++)
      {
	if(buff[j]==',')
	{
	  cout<<"那个逗号是英文格式的"<<endl;
	}
      }
      */
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      //如果是字符串用此方法显示 
      printf("\n%s",buff);
      tcflush(serial_fd,TCIFLUSH);
      ///////////////////////////////////////////////////////////////////////////////////////////////
      //Modified by Tianhao Wang
      //cope with the specific condition that acc sometimes is a minus number 
      //sometimes euler angle is a three-digit number and sometimes euler angle is a two-digit number
      if(buff[0]=='-')
      {
	 
	    buff_acc_x[0]=buff[0];
	    buff_acc_x[1]=buff[1];
	    buff_acc_x[2]=buff[2];
	    buff_acc_x[3]=buff[3];
	    buff_acc_x[4]=buff[4];
	    
	    if(buff[7]=='-')
	    {
	      buff_acc_y[0]=buff[7];
	      buff_acc_y[1]=buff[8];
	      buff_acc_y[2]=buff[9];
	      buff_acc_y[3]=buff[10];
	      buff_acc_y[4]=buff[11];
	      
		  if(buff[14]=='-')
		  {
		    int i=21;
		    //I am sure that under this condition, the 21th character 
		    // Modified by Tianhao Wang on 3th,May
		    //I added an ending flag, which bring great convenience
			while(buff[i]!=',')
			{
			  buff_pitch[i-21]=buff[i];
			  i++;
			}
			i=i+2;
			int temp=i;
			while(buff[i]!=',')
			{
			  buff_yaw[i-temp]=buff[i];
			  i++;
			}
		        i=i+2;
			temp=i;
			while(i<=longth-1)
			{
			  buff_roll[i-temp]=buff[i];
			  i++;
			} 
		  }  
		  else
		  {
		      int i=20;
		      //I am sure that under this condition, the 21th character 
		      // Modified by Tianhao Wang on 3th,May
		      //I added an ending flag, which bring great convenience
			  while(buff[i]!=',')
			  {
			    buff_pitch[i-20]=buff[i];
			    i++;
			  }
			  i=i+2;
			  int temp=i;
			  while(buff[i]!=',')
			  {
			    buff_yaw[i-temp]=buff[i];
			    i++;
			  }
			  i=i+2;
			  temp=i;
			  while(i<=longth-1)
			  {
			    buff_roll[i-temp]=buff[i];
			    i++;
			  } 
		  }
	    }
	    else
	    {
	      buff_acc_y[0]=buff[7];
	      buff_acc_y[1]=buff[8];
	      buff_acc_y[2]=buff[9];
	      buff_acc_y[3]=buff[10];
		
		  if(buff[13]=='-')
		  {
		    int i=20;
		    //I am sure that under this condition, the 21th character 
		    // Modified by Tianhao Wang on 3th,May
		    //I added an ending flag, which bring great convenience
			while(buff[i]!=',')
			{
			  buff_pitch[i-20]=buff[i];
			  i++;
			}
			i=i+2;
			int temp=i;
			while(buff[i]!=',')
			{
			  buff_yaw[i-temp]=buff[i];
			  i++;
			}
		        i=i+2;
			temp=i;
			while(i<=longth-1)
			{
			  buff_roll[i-temp]=buff[i];
			  i++;
			} 
		  }
		  else
		  {
		      int i=19;
		      //I am sure that under this condition, the 21th character 
		      // Modified by Tianhao Wang on 3th,May
		      //I added an ending flag, which bring great convenience
			  while(buff[i]!=',')
			  {
			    buff_pitch[i-19]=buff[i];
			    i++;
			  }
			  i=i+2;
			  int temp=i;
			  while(buff[i]!=',')
			  {
			    buff_yaw[i-temp]=buff[i];
			    i++;
			  }
			  i=i+2;
			  temp=i;
			  while(i<=longth-1)
			  {
			    buff_roll[i-temp]=buff[i];
			    i++;
			  } 
		  }
              }
      }
      //the first character is not '-'
      else
      {
	buff_acc_x[0]=buff[0];
	buff_acc_x[1]=buff[1];
	buff_acc_x[2]=buff[2];
	buff_acc_x[3]=buff[3];
	
	if(buff[6]=='-')
	{
	  buff_acc_y[0]=buff[6];
	  buff_acc_y[1]=buff[7];
	  buff_acc_y[2]=buff[8];
	  buff_acc_y[3]=buff[9];
	  buff_acc_y[4]=buff[10];
	   
	      if(buff[13]=='-')
	      {
		  int i=20;
		    //I am sure that under this condition, the 21th character 
		    // Modified by Tianhao Wang on 3th,May
		    //I added an ending flag, which bring great convenience
			while(buff[i]!=',')
			{
			  buff_pitch[i-20]=buff[i];
			  i++;
			}
			i=i+2;
			int temp=i;
			while(buff[i]!=',')
			{
			  buff_yaw[i-temp]=buff[i];
			  i++;
			}
		        i=i+2;
			temp=i;
			while(i<=longth-1)
			{
			  buff_roll[i-temp]=buff[i];
			  i++;
			} 
	      }  
	      else
	      {
		   int i=19;
		      //I am sure that under this condition, the 21th character 
		      // Modified by Tianhao Wang on 3th,May
		      //I added an ending flag, which bring great convenience
			  while(buff[i]!=',')
			  {
			    buff_pitch[i-19]=buff[i];
			    i++;
			  }
			  i=i+2;
			  int temp=i;
			  while(buff[i]!=',')
			  {
			    buff_yaw[i-temp]=buff[i];
			    i++;
			  }
			  i=i+2;
			  temp=i;
			  while(i<=longth-1)
			  {
			    buff_roll[i-temp]=buff[i];
			    i++;
			  } 
	      }
	}
	else
	{
	  buff_acc_y[0]=buff[6];
	  buff_acc_y[1]=buff[7];
	  buff_acc_y[2]=buff[8];
	  buff_acc_y[3]=buff[9];
	    
	      if(buff[12]=='-')
	      {
		  int i=19;
		      //I am sure that under this condition, the 21th character 
		      //while(buff[i]!='*')// Modified by Tianhao Wang on 3th,May
		      //I added an ending flag, which bring great convenience
			  while(buff[i]!=',')
			  {
			    buff_pitch[i-19]=buff[i];
			    i++;
			  }
			  i=i+2;
			  int temp=i;
			  while(buff[i]!=',')
			  {
			    buff_yaw[i-temp]=buff[i];
			    i++;
			  }
			  i=i+2;
			  temp=i;
			  while(i<=longth-1)
			  {
			    buff_roll[i-temp]=buff[i];
			    i++;
			  } 
		    
	      }  
	      else
	      {
		   int i=18;
		      //I am sure that under this condition, the 21th character 
		      // Modified by Tianhao Wang on 3th,May
		      //I added an ending flag, which bring great convenience
			  while(buff[i]!=',')
			  {
			    buff_pitch[i-18]=buff[i];
			    i++;
			  }
			  i=i+2;
			  int temp=i;
			  while(buff[i]!=',')
			  {
			    buff_yaw[i-temp]=buff[i];
			    i++;
			  }
			  i=i+2;
			  temp=i;
			  while(i<=longth-1)
			  {
			    buff_roll[i-temp]=buff[i];
			    i++;
			  } 
	      }
	}
      }
      //Finish serial read part;       
      ///////////////////////////////////////////////////////////////////////////////////////////////
     
      //prepared for convert to float
      sta_pitch=atof(buff_pitch);
      sta_yaw=atof(buff_yaw);
      sta_roll=atof(buff_roll); 
      sta_acc_x=atof(buff_acc_x);
      sta_acc_y=atof(buff_acc_y);
      //test
      /*
      std::cout<<sta_pitch<<std::endl;
      std::cout<<sta_yaw<<std::endl;
      std::cout<<sta_roll<<std::endl;
      std::cout<<sta_acc_x<<std::endl;
      std::cout<<sta_acc_y<<std::endl;
      */
      
      //清空数组 
     bzero(buff,1024);
     rvec.at<float>(0, 0) += CV_PI *((sta_pitch-previous_pitch)/360);
     rvec.at<float>(0, 1) += CV_PI *((sta_yaw-previous_yaw)/360); 
     rvec.at<float>(0, 2) += CV_PI *((sta_roll-previous_roll)/360); 
    
     cv::Mat rmat; 
     //罗德里格斯公式，将罗德里格斯向量转换成旋转矩阵 
     Rodrigues(rvec, rmat); 
     //构造仿射变换类型的pose，这个类型暂且看做OpenCV中的位姿类型，两个参数，一个旋转，一个平移 
     /////////////////////////////////////////////////////////////////////////
     //Modified by Tianhao Wang,I want to use 
     /////////////////////////////////////////////////////////////////////////
     
     Affine3f pose(rmat, Vec3f(0, 0, 0)); 
     //这一句就是整个可视化窗口能够动起来的核心语句了， 
     //说白了就是利用循环来不断调整上面plane部件的位姿，达到动画的效果 
     //另外这里就利用到了平面的ID，来表征调整的是平面的位姿 
     //window.setWidgetPose("plane", pose); 
     //window.spinOnce(1, false);
     //控制单帧暂留时间，调整time参数的效果就是平面转的快慢，本质上就是每一个位姿的停留显示时间。 
     vis_IMU.setWidgetPose ( "IMU", pose );
     vis_IMU.spinOnce(1, false);
     imu1.calintegral(sta_acc_x);
     imu2.calintegral(sta_acc_y);
     
     std::cout<<imu1.flag_immobile<<std::endl;
     std::cout<<imu2.flag_immobile<<std::endl;
     
     if((sta_acc_x-previous_acc_x>0.03)||(sta_acc_y-previous_acc_y>0.03)||(sta_pitch-previous_pitch>1)||(sta_yaw-previous_yaw>1)||(sta_roll-previous_roll>1))
     {
       immobile_pic_flag.push_back(1);
       std::cout<<"动了"<<std::endl;
     }
     else
     {
       immobile_pic_flag.push_back(0);
       std::cout<<"没动"<<std::endl;
     }
     
     previous_pitch=sta_pitch;
     previous_yaw=sta_yaw;
     previous_roll=sta_roll;
     previous_acc_x=sta_acc_x;
     previous_acc_y=sta_acc_y;
     
     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
     std::cout<<time_used.count()<<std::endl;
    }
    else
    {
      usleep(1); 
    }
  }
  //关闭串口文件 
  close(serial_fd);
  exit(0);
}

float calDistance(float coor_x,float coor_y,float A,float B,float C)
{
  float temp(0);
  temp=fabs(coor_x*A+coor_y*B+C);
  return temp/(sqrt(A*A+B*B));
}


void LK_pyramid_FAST_method()
{
  string path_to_dataset="/home/wth/design/Myslam/SLAM_1.0_demo3/dataset_for_demo";
   // string rgb_file, depth_file, time_rgb, time_depth;
    list< cv::Point2f > keypoints;      
    // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    cv::Mat fundamental_matrix;
    //declare the mat ahead of time
    for ( int index=0; index<8; index++ )
    {
        //fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
	
	boost::format fmt( "%s%d.%s" );
        color=cv::imread((fmt%"/home/wth/design/Myslam/SLAM_1.0_demo3/dataset_for_demo/bgr/bgr"%(index+24)%"jpg").str());
        depth=cv::imread((fmt%"/home/wth/design/Myslam/SLAM_1.0_demo3/dataset_for_demo/newdepth/depthg"%(index+1)%"png").str(),-1);
        
	if (index ==0 )
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps );
            for ( auto kp:kps )
	    {
              keypoints.push_back( kp.pt );
	    }
	    last_color = color;
            continue;
        }
        
        if ( color.data==nullptr || depth.data==nullptr )
	{
            continue;
	}
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
	list<cv::Point2f> prev_keypoints_correspond;
	vector<cv::Point2f> prev_keypoints_correspond_vector;
	vector<cv::Point2f> keypoints_vector;
        for ( auto kp:keypoints )
	{
            prev_keypoints.push_back(kp);
	    
	}
	vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	//*******************************************************************************************
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        //*******************************************************************************************
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	std:cout<<"LK Flow use time:"<<time_used.count()<<"seconds"<<std::endl;
	
        // 把跟丢的点删掉
	//这一段是非常关键的代码
	/////////////////////////////////////////
	//Modified by Tianhao Wang at 2019.3.30
	 for ( auto kp:keypoints )
	{
            prev_keypoints_correspond.push_back(kp);
	    
	}
	////////////////////////////////////////
        int i=0; 
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
	  
            if ( status[i] == 0 )
            {
                iter = keypoints.erase(iter);
                continue;
            }
            
            *iter = next_keypoints[i];
            iter++;
	    //And in this step, we also set next_keypoints got just now as key_points in the new iteration
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Modified by Tianhao Wang, I want to delete the points in prev_keypoints which are lost in next_keypoints.
        //And prev_keypoints_correspond is used to draw the points in image and explicitly show the matching result.
        //I have deleted all points we lost track in prev_keypoints_correspond
        i=0;
         for ( auto iter_p=prev_keypoints_correspond.begin(); iter_p!=prev_keypoints_correspond.end(); i++)
        {
	  
            if ( status[i] == 0 )
            {
                iter_p = prev_keypoints_correspond.erase(iter_p);
                continue;
            }           
            iter_p++;
        }
        //////////////////////////////////////////////////////////////////////////////////
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone();
	cv::Mat img_show_original=last_color.clone();
	int flag(0);
        for ( auto kp:keypoints )
	{
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
	  //Only show one key_point to check whether we have tracked it.
	}
        for ( auto kp:prev_keypoints_correspond )
	{
            cv::circle(img_show_original, kp, 10, cv::Scalar(0, 0, 240), 1);
	  //Only show one key_point to check whether we have tracked it.
	}
        /*for(int i(0);i<next_keypoints.size();i++)
	{
	if(status[i]==0)
	  {
	    cv::circle(img_show, next_keypoints[i], 10, cv::Scalar(255, 0, 0), 1);
	  }
	 else
	 {
	   cv::circle(img_show, next_keypoints[i], 10, cv::Scalar(0, 240, 0), 1);
	 }
	}
	*/
	
	
        //Modified by Tianhao Wang on 2019.4.6, calculate the fundamental matrix
        //the function for fundamental matrix requires vector as input, in other words cv::inputarray can't be a list
	for ( auto kp:prev_keypoints_correspond )
	{
            prev_keypoints_correspond_vector.push_back(kp);
	}
	for ( auto kp:keypoints )
	{
            keypoints_vector.push_back(kp);
	}
        fundamental_matrix=cv::findFundamentalMat(prev_keypoints_correspond_vector,keypoints_vector,CV_FM_RANSAC);
	std::cout<<"fundamental matrix is as follows"<<std::endl<<fundamental_matrix<<std::endl;
	
	cv::destroyAllWindows();
	cv::imshow("corners", img_show);
	cv::imshow("last_corners",img_show_original);
        cv::waitKey(3000);
	//I successfully get the fundamental matrix
	
	
	/*
	std::cout<<"the first data in initial FAST keypoint is"<<prev_keypoints[0]<<std::endl;
	std::cout<<"the second data in initial FAST keypoint is"<<prev_keypoints[1]<<std::endl;
	std::cout<<"the first data in prev_keypoints is"<<prev_keypoints[0]<<std::endl;
	std::cout<<"the second data in prev_keypoints is"<<prev_keypoints[1]<<std::endl;
	std::cout<<"the first data in next_keypoints is"<<next_keypoints[0]<<std::endl;
	std::cout<<"the second data in next_keypoints is"<<next_keypoints[1]<<std::endl;
        //This part is to test whethe calcOpticalFlowPyrLK can give a tracking result 
	*/
	
	/*
	//Modified 
	//Due to the restriction of the KeyPoint, I use convert function;
	cv::Mat match_result;
	std::vector <cv::KeyPoint> key_1,key_2;
	cv::KeyPoint::convert(prev_keypoints_correspond_vector,key_1,1,1,0,-1);
        cv::KeyPoint::convert(keypoints_vector,key_2,1,1,0,-1);
	cv::drawMatches(img_show_original,key_1,img_show,key_2,match_result);
	imshow(" 匹配后的结果",match_result);
	cv::waitKey(3000);
	//
	*/
	
	//Modified by Tianhao Wang on 2019.4.7, this part is used to calculate the epilines
	std::vector<cv::Vec<float,3>> epilines1,epilines2;
	cv::computeCorrespondEpilines(prev_keypoints_correspond_vector,1,fundamental_matrix,epilines2);
	cv::computeCorrespondEpilines(keypoints_vector,2,fundamental_matrix,epilines1);
	//the integer(the second input) indicates which img the points set belongs to.
	//Attention:epilines2, which is in the current frame is obtained according to the points set in previous frame. 
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	cv::RNG rng(time(0));
	
	for(uint i(0);i<keypoints_vector.size();i++)
	{
	  cv::Scalar color=cv::Scalar(rng(256),rng(256),rng(256)); 
	  cv::line(img_show, cv::Point(0, -epilines1[i][2] / epilines1[i][1]), cv::Point(img_show.cols, -(epilines1[i][2] + epilines1[i][0] * img_show.cols) / epilines1[i][1]), color);
          //绘制外极线的时候，选择两个点，一个是x=0处的点，一个是x为图片宽度处
          cv::line(img_show_original, cv::Point(0, -epilines2[i][2] / epilines2[i][1]), cv::Point(img_show_original.cols, -(epilines2[i][2] + epilines2[i][0] * img_show_original.cols) / epilines2[i][1]), color);
	}
	
	cv::destroyWindow("corners");
	cv::destroyWindow("last_corners");
	cv::imshow("corresponding epilines1",img_show);
	cv::imshow("corresponding epilines2",img_show_original);
	cv::waitKey(3000);
	
	//Modified by Tianhao Wang, I begin to calculate the distance from matched points to their corresponding epilines;
	int count=keypoints_vector.size();
	float distance_ce[count];
	cv::Mat img_result=img_show.clone();
	for(uint i(0);i<keypoints_vector.size();i++)
	{
	 distance_ce[i]=calDistance(keypoints_vector[i].x,keypoints_vector[i].y,epilines2[i][0],epilines2[i][1],epilines2[i][2]); 
	 //std::cout<<"距对应的极线距离是"<<distance_ce[i]<<std::endl;
	 if(distance_ce[i]>100)
	 {
	   //I set threshold 15 first.
	     cv::circle(img_result, keypoints_vector[i], 10, cv::Scalar(240, 0, 0), 1);
	 }
	}
	cv::destroyAllWindows();
	cv::imshow("final_result",img_result);
	cv::waitKey(3000);
	//sort part and get the threshold(how to get the threshold?)
	//
	cv::imwrite((fmt%"/home/wth/design/test_track/bgr"%(index)%"jpg").str(),img_show);
	cv::imwrite((fmt%"/home/wth/design/test_track/bgr_original"%(index)%"jpg").str(),img_show_original);
	cv::imwrite((fmt%"/home/wth/design/test_track/bgr_final_result"%(index)%"jpg").str(),img_result);
	last_color = color;
    }
}


cv::Mat IMU_twc_estimation(int frame_i,int frame_j)
{
        boost::format fmt( "%s%d.%s" );
        string file_path1=(fmt%"/home/wth/design/Myslam/dataset/bgr/bgr_log"%(frame_i)%"jpg").str();
	string file_path2=(fmt%"/home/wth/design/Myslam/dataset/bgr/bgr_log"%(frame_j)%"jpg").str();
	
        ifstream in1(file_path1);
	ifstream in2(file_path2);
	
        string line_file1;
	string line_file2;
	double sta_pitch(0);
        double sta_yaw(0);
	double sta_roll(0);
	double previous_pitch(0);
	double previous_yaw(0);
	double previous_roll(0);
	char char_pitch_2[7];
	char char_yaw_2[7];
	char char_roll_2[7];
	char char_pitch_1[7];
	char char_yaw_1[7];
	char char_roll_1[7];
	
	if(in1) 
	{
		while (getline (in1, line_file1)) 
		  // line中不包括每行的换行符
		{ 
		  cout<<line_file1<<endl;
		  //convert string to float
		}
	}
	else 
	{
		cout <<"no such file1" << endl;
	}
	
	if(in2) 
	{
		while (getline (in2, line_file2)) 
		  // line中不包括每行的换行符
		{ 
		 cout<<line_file2<<endl; 
		}
	}
	else 
	{
		cout <<"no such file2" << endl;
	}
	int flag_count=0;
	while(1)
	{
	  int flag_c=0;
	  while(line_file1[flag_count]!='*')
	  {
	    char_pitch_1[flag_c]=line_file1[flag_count];
	    flag_count++;
	  }
	  flag_count++;
	  flag_c=0;
	   while(line_file1[flag_count]!='*')
	  {
	    char_yaw_1[flag_c]=line_file1[flag_count];
	    flag_count++;
	  }
	  flag_count++;
	  flag_c=0;
	  while(line_file1[flag_count]!='#')
	  {
	    char_roll_1[flag_c]=line_file1[flag_count];
	    flag_count++;
	  }
	  if(line_file1[flag_count]=='#')
	  {
	    break;
	  }
	}
	flag_count=0;
	
	while(1)
	{
	  int flag_c=0;
	  while(line_file2[flag_count]!='*')
	  {
	    char_pitch_2[flag_c]=line_file2[flag_count];
	    flag_count++;
	  }
	  flag_count++;
	  flag_c=0;
	   while(line_file2[flag_count]!='*')
	  {
	    char_yaw_2[flag_c]=line_file2[flag_count];
	    flag_count++;
	  }
	  flag_count++;
	  flag_c=0;
	  while(line_file2[flag_count]!='#')
	  {
	    char_roll_2[flag_c]=line_file2[flag_count];
	    flag_count++;
	  }
	  if(line_file2[flag_count]=='#')
	  {
	    break;
	  }
	}
	
	sta_pitch=atof(char_pitch_2);
        sta_yaw=atof(char_yaw_2);
	sta_roll=atof(char_roll_2);
	previous_pitch=atof(char_pitch_1);
	previous_yaw=atof(char_yaw_1);
	previous_roll=atof(char_roll_1);
	
        cv::Mat R_mat_X=(cv::Mat_<float>(3,3) << 1,0,0,0,cos(CV_PI*(sta_pitch-previous_pitch/180)),-sin(CV_PI*(sta_pitch-previous_pitch/180)),0,sin(CV_PI*(sta_pitch-previous_pitch/180)),cos(CV_PI*(sta_pitch-previous_pitch/180))); 
        cv::Mat R_mat_Y=(cv::Mat_<float>(3,3) << cos(CV_PI*(sta_yaw-previous_yaw/180)),0,sin(CV_PI*(sta_yaw-previous_yaw/180)),0,1,0,-sin(CV_PI*(sta_yaw-previous_yaw/180)),0,cos(CV_PI*(sta_yaw-previous_yaw/180))); 
        cv::Mat R_mat_Z=(cv::Mat_<float>(3,3) << cos(CV_PI*(sta_roll-previous_roll/180)),-sin(CV_PI*(sta_roll-previous_roll/180)),0,sin(CV_PI*(sta_roll-previous_roll/180)),cos(CV_PI*(sta_roll-previous_roll/180)),0,0,0,1); 
        cv::Mat final_rotation;
	final_rotation=R_mat_X*R_mat_Y*R_mat_Z;
	
	return final_rotation;
}
//return a cv matrix(rotation part os the transform matrix converted from Euler angle)



int main( int argc, char** argv )
{
  /*
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file );
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    */
    
   /**************OPENNI接口读取Kinect*******************/
	//打开串口
	
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
    mModeDepth.setFps( 10);
    // 像素格式
    mModeDepth.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );

    streamDepth.setVideoMode( mModeDepth);
    
    // 同样的设置彩色图像视频模式
    openni::VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 10 );
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
    ////////////////////////////////////////////////////////////////////////////////////////
    //combine imu with kinect to capture pictures; 
    init_serial();
    //设置接收缓冲区的大小
    char buff[1024];
    char end_flag[1];
    end_flag[0]='*';

    //可以一个一个字节的接收 
    char uart_0=0;
    char buff_pitch[6];
    char buff_yaw[6];
    char buff_roll[6];
    char buff_acc_x[6];
    char buff_acc_y[6];
    char buff_acc_z[6];
    int nread;
    double sta_pitch(0);
    double previous_pitch(0);
    double sta_yaw(0);
    double previous_yaw(0);
    double sta_roll(0);
    double previous_roll(0);
    double sta_acc_x(0);
    double previous_acc_x(0);
    double sta_acc_y(0);
    double previous_acc_y(0);
    int initial_flag(0);
    double ini_pitch(0);
    double ini_yaw(0);
    double ini_roll(0);
    
    viz::Viz3d window("IMU window"); 
    window.showWidget("Coordinate", viz::WCoordinateSystem()); 
    //创建平面 
    viz::WPlane plane; 
    //添加平面，并设置一个ID为plane 
    window.showWidget("plane", plane); 
    
    
    //创建一个1*3的rotation vector 
    Mat rvec = Mat::zeros(1, 3, CV_32F); 
    //Modified by Tianhao Wang on May,8th
    //new visualization
    // visualization
    cv::viz::Viz3d vis_IMU ( "IMU" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis_IMU.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis_IMU.showWidget ( "World", world_coor );
    vis_IMU.showWidget ( "IMU", camera_coor );
	
    //myslam::IMU_integration imu1;
    //myslam::IMU_integration imu2;
  //
   // 创建OpenCV图像窗口
    cv::namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );
    int flag(0);
    int save_flag(0);
    Affine3f pose;
  while(!vis_IMU.wasStopped())
  {
    chrono::steady_clock::time_point t11 = chrono::steady_clock::now();
    if(cv::waitKey(1)=='j')
    {
      break;
    }
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
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
	//mScaledDepth is for mapping while mScaledDepth is for display
	imshow("Color Image",cImageBGR);
	imshow("Depth Image",mScaledDepth);
	cv::waitKey(1);
	
	boost::format fmt("%s%d.%s");
	cv::Mat flipmImageDepth;
	cv::flip(mImageDepth,flipmImageDepth,1);
	
     chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
     chrono::duration<double> time_used_capture = chrono::duration_cast<chrono::duration<double>>( t4-t3 );
     std::cout<<"采集图像所用的时间"<<time_used_capture.count()<<std::endl;
     //
    //如果串口缓冲区中有数据 
    if((nread = read(serial_fd,buff,1024))>0)
    {
      //strcat(buff,end_flag);
      int longth=strlen(buff);
      cout<<"长度是"<<longth<<endl;
      /*for(int j(0);j<=longth-1;j++)
      {
	if(buff[j]==',')
	{
	  cout<<"那个逗号是英文格式的"<<endl;
	}
      }
      */
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      //如果是字符串用此方法显示 
      printf("\n%s",buff);
      tcflush(serial_fd,TCIFLUSH);
      ///////////////////////////////////////////////////////////////////////////////////////////////
      //Modified by Tianhao Wang
      //cope with the specific condition that acc sometimes is a minus number 
      //sometimes euler angle is a three-digit number and sometimes euler angle is a two-digit number
	  if(buff[0]=='-')
	  {
	    
		buff_acc_x[0]=buff[0];
		buff_acc_x[1]=buff[1];
		buff_acc_x[2]=buff[2];
		buff_acc_x[3]=buff[3];
		buff_acc_x[4]=buff[4];
		
		if(buff[7]=='-')
		{
		  buff_acc_y[0]=buff[7];
		  buff_acc_y[1]=buff[8];
		  buff_acc_y[2]=buff[9];
		  buff_acc_y[3]=buff[10];
		  buff_acc_y[4]=buff[11];
		  
		      if(buff[14]=='-')
		      {
			int i=21;
			//I am sure that under this condition, the 21th character 
			// Modified by Tianhao Wang on 3th,May
			//I added an ending flag, which bring great convenience
			    while(buff[i]!=',')
			    {
			      buff_pitch[i-21]=buff[i];
			      i++;
			    }
			    i=i+2;
			    int temp=i;
			    while(buff[i]!=',')
			    {
			      buff_yaw[i-temp]=buff[i];
			      i++;
			    }
			    i=i+2;
			    temp=i;
			    while(i<=longth-1)
			    {
			      buff_roll[i-temp]=buff[i];
			      i++;
			    } 
		      }  
		      else
		      {
			  int i=20;
			  //I am sure that under this condition, the 21th character 
			  // Modified by Tianhao Wang on 3th,May
			  //I added an ending flag, which bring great convenience
			      while(buff[i]!=',')
			      {
				buff_pitch[i-20]=buff[i];
				i++;
			      }
			      i=i+2;
			      int temp=i;
			      while(buff[i]!=',')
			      {
				buff_yaw[i-temp]=buff[i];
				i++;
			      }
			      i=i+2;
			      temp=i;
			      while(i<=longth-1)
			      {
				buff_roll[i-temp]=buff[i];
				i++;
			      } 
		      }
		}
		else
		{
		  buff_acc_y[0]=buff[7];
		  buff_acc_y[1]=buff[8];
		  buff_acc_y[2]=buff[9];
		  buff_acc_y[3]=buff[10];
		    
		      if(buff[13]=='-')
		      {
			int i=20;
			//I am sure that under this condition, the 21th character 
			// Modified by Tianhao Wang on 3th,May
			//I added an ending flag, which bring great convenience
			    while(buff[i]!=',')
			    {
			      buff_pitch[i-20]=buff[i];
			      i++;
			    }
			    i=i+2;
			    int temp=i;
			    while(buff[i]!=',')
			    {
			      buff_yaw[i-temp]=buff[i];
			      i++;
			    }
			    i=i+2;
			    temp=i;
			    while(i<=longth-1)
			    {
			      buff_roll[i-temp]=buff[i];
			      i++;
			    } 
		      }
		      else
		      {
			  int i=19;
			  //I am sure that under this condition, the 21th character 
			  // Modified by Tianhao Wang on 3th,May
			  //I added an ending flag, which bring great convenience
			      while(buff[i]!=',')
			      {
				buff_pitch[i-19]=buff[i];
				i++;
			      }
			      i=i+2;
			      int temp=i;
			      while(buff[i]!=',')
			      {
				buff_yaw[i-temp]=buff[i];
				i++;
			      }
			      i=i+2;
			      temp=i;
			      while(i<=longth-1)
			      {
				buff_roll[i-temp]=buff[i];
				i++;
			      } 
		      }
		  }
	  }
	  //the first character is not '-'
	  else
	  {
	    buff_acc_x[0]=buff[0];
	    buff_acc_x[1]=buff[1];
	    buff_acc_x[2]=buff[2];
	    buff_acc_x[3]=buff[3];
	    
	    if(buff[6]=='-')
	    {
	      buff_acc_y[0]=buff[6];
	      buff_acc_y[1]=buff[7];
	      buff_acc_y[2]=buff[8];
	      buff_acc_y[3]=buff[9];
	      buff_acc_y[4]=buff[10];
	      
		  if(buff[13]=='-')
		  {
		      int i=20;
			//I am sure that under this condition, the 21th character 
			// Modified by Tianhao Wang on 3th,May
			//I added an ending flag, which bring great convenience
			    while(buff[i]!=',')
			    {
			      buff_pitch[i-20]=buff[i];
			      i++;
			    }
			    i=i+2;
			    int temp=i;
			    while(buff[i]!=',')
			    {
			      buff_yaw[i-temp]=buff[i];
			      i++;
			    }
			    i=i+2;
			    temp=i;
			    while(i<=longth-1)
			    {
			      buff_roll[i-temp]=buff[i];
			      i++;
			    } 
		  }  
		  else
		  {
		      int i=19;
			  //I am sure that under this condition, the 21th character 
			  // Modified by Tianhao Wang on 3th,May
			  //I added an ending flag, which bring great convenience
			      while(buff[i]!=',')
			      {
				buff_pitch[i-19]=buff[i];
				i++;
			      }
			      i=i+2;
			      int temp=i;
			      while(buff[i]!=',')
			      {
				buff_yaw[i-temp]=buff[i];
				i++;
			      }
			      i=i+2;
			      temp=i;
			      while(i<=longth-1)
			      {
				buff_roll[i-temp]=buff[i];
				i++;
			      } 
		  }
	    }
	    else
	    {
	      buff_acc_y[0]=buff[6];
	      buff_acc_y[1]=buff[7];
	      buff_acc_y[2]=buff[8];
	      buff_acc_y[3]=buff[9];
		
		  if(buff[12]=='-')
		  {
		      int i=19;
			  //I am sure that under this condition, the 21th character 
			  //while(buff[i]!='*')// Modified by Tianhao Wang on 3th,May
			  //I added an ending flag, which bring great convenience
			      while(buff[i]!=',')
			      {
				buff_pitch[i-19]=buff[i];
				i++;
			      }
			      i=i+2;
			      int temp=i;
			      while(buff[i]!=',')
			      {
				buff_yaw[i-temp]=buff[i];
				i++;
			      }
			      i=i+2;
			      temp=i;
			      while(i<=longth-1)
			      {
				buff_roll[i-temp]=buff[i];
				i++;
			      } 
			
		  }  
		  else
		  {
		      int i=18;
			  //I am sure that under this condition, the 21th character 
			  // Modified by Tianhao Wang on 3th,May
			  //I added an ending flag, which bring great convenience
			      while(buff[i]!=',')
			      {
				buff_pitch[i-18]=buff[i];
				i++;
			      }
			      i=i+2;
			      int temp=i;
			      while(buff[i]!=',')
			      {
				buff_yaw[i-temp]=buff[i];
				i++;
			      }
			      i=i+2;
			      temp=i;
			      while(i<=longth-1)
			      {
				buff_roll[i-temp]=buff[i];
				i++;
			      } 
		  }
	    }
	  }
	  //Finish serial read part;       
	  ///////////////////////////////////////////////////////////////////////////////////////////////
	
	  //prepared for convert to float
	  sta_pitch=atof(buff_pitch);
	  sta_yaw=atof(buff_yaw);
	  sta_roll=atof(buff_roll); 
	  sta_acc_x=atof(buff_acc_x);
	  sta_acc_y=atof(buff_acc_y);
	  //test
	  /*
	  std::cout<<sta_pitch<<std::endl;
	  std::cout<<sta_yaw<<std::endl;
	  std::cout<<sta_roll<<std::endl;
	  std::cout<<sta_acc_x<<std::endl;
	  std::cout<<sta_acc_y<<std::endl;
	  */
	  
	  //清空数组 
	bzero(buff,1024);
	
        cv::Mat R_mat_X=(cv::Mat_<float>(3,3) << 1,0,0,0,cos(CV_PI*(sta_pitch-previous_pitch/180)),-sin(CV_PI*(sta_pitch-previous_pitch/180)),0,sin(CV_PI*(sta_pitch-previous_pitch/180)),cos(CV_PI*(sta_pitch-previous_pitch/180))); 
        cv::Mat R_mat_Y=(cv::Mat_<float>(3,3) << cos(CV_PI*(sta_yaw-previous_yaw/180)),0,sin(CV_PI*(sta_yaw-previous_yaw/180)),0,1,0,-sin(CV_PI*(sta_yaw-previous_yaw/180)),0,cos(CV_PI*(sta_yaw-previous_yaw/180))); 
        cv::Mat R_mat_Z=(cv::Mat_<float>(3,3) << cos(CV_PI*(sta_roll-previous_roll/180)),-sin(CV_PI*(sta_roll-previous_roll/180)),0,sin(CV_PI*(sta_roll-previous_roll/180)),cos(CV_PI*(sta_roll-previous_roll/180)),0,0,0,1); 
        cv::Mat final_rotation;
	final_rotation=R_mat_X*R_mat_Y*R_mat_Z;
	
	cout<<final_rotation<<endl;
	Affine3f pose_now;
	if(flag==0)
	{
	  Affine3f pose_temp(final_rotation,cv::Vec3f(1,1,1));
	  pose_now=pose_temp;
	}
	else
	{
	  Affine3f pose_regu(final_rotation,cv::Vec3f(0,0,0));
	  pose_now=pose_regu;
	}
	Affine3f M;
	M=pose*pose_now;
	pose=M;
	
	//imu1.calintegral(sta_acc_x);
	//imu2.calintegral(sta_acc_y);
	//std::cout<<imu1.flag_immobile<<std::endl;
	//std::cout<<imu2.flag_immobile<<std::endl;
	if((sta_acc_x-previous_acc_x>0.3)||(sta_acc_y-previous_acc_y>0.3)||(sta_pitch-previous_pitch>1)||(sta_yaw-previous_yaw>1)||(sta_roll-previous_roll>1))
	{
	  immobile_pic_flag.push_back(1);
	  //std::cout<<"动了"<<std::endl;
	}
	else
	{
	  immobile_pic_flag.push_back(0);
	  //std::cout<<"没动"<<std::endl;
	}
	previous_pitch=sta_pitch;
	previous_yaw=sta_yaw;
	previous_roll=sta_roll;
	previous_acc_x=sta_acc_x;
	previous_acc_y=sta_acc_y;
	
	
	 if((flag%3)==0)
	{
	
	//pose的定义在大循环之外
	vis_IMU.setWidgetPose ( "IMU", M );
	vis_IMU.spinOnce(10, false);
	cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/bgr/bgr"%(save_flag)%"jpg").str(),cImageBGR);
	cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/depth/depth"%(save_flag)%"jpg").str(),mImageDepth);
	cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/depth/depth"%(save_flag)%"png").str(),mImageDepth);
	cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/depth_flip/depth_flip"%(save_flag)%"jpg").str(),flipmImageDepth);
	cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/depth_flip/depth_flip"%(save_flag)%"png").str(),flipmImageDepth);
    
        //在这里记录下来这一帧当前的欧拉角数值，然后记录到log里面去
	
	FILE *fpp=NULL;
	string temp;
	temp=(fmt%"/home/wth/design/Myslam/dataset/bgr_log/bgr_log"%(save_flag)%"txt").str();
        int ci=temp.size();
	char temp_c[ci];
        temp.copy(temp_c,ci,0);
	*(temp_c+ci)='\0';
	fpp=fopen(temp_c,"w");
	
	boost::format fmt_log( "%d*%d*%d#\n" );
	string temp_log=(fmt_log%sta_pitch%sta_yaw%sta_roll).str();
	
	 int cl=temp_log.size();
	char temp_l[cl];
        temp_log.copy(temp_l,cl,0);
	*(temp_l+cl)='\0';
	
        fprintf(fpp,temp_l);
        fclose(fpp); 
	
	save_flag++;
	}
	flag++;
	
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	std::cout<<"计算其是否动的时间"<<time_used.count()<<std::endl;
    }
    else
    {
      usleep(1); 
    } 
     chrono::steady_clock::time_point t22 = chrono::steady_clock::now();
     chrono::duration<double> time_used_all = chrono::duration_cast<chrono::duration<double>>( t22-t11 );
     std::cout<<" 整个循环所耗的时间"<<time_used_all.count()<<std::endl;
     
     if(flag==24)
     {
      break; 
     }
  }//这里是最外层的while的循环的括号
   
    //关闭串口文件 
    close(serial_fd);
    ////////////////////////////////////////////////////////////////////////////////////////
        // 关闭数据流
    streamDepth.destroy();
    streamColor.destroy();
    // 关闭设备q
    devAnyDevice.close();
    // 最后关闭OpenNI
    cv::destroyAllWindows();
    openni::OpenNI::shutdown();
/**************************************************/
    ///////////////////////////////////////////////////////////////////////
    //Modified by Tianhao Wang, using shell to use python demo.py
    //before this step: I have acquire and depth images,and immobile flag vector
    FILE *fp=NULL; 
    FILE *fh=NULL; 
    char buff_system[128]={0};   
   memset(buff_system,0,sizeof(buff_system));
   fp=popen("python /home/wth/Mask_RCNN/samples/demo.py","r");
   //将命令ls-l 同过管道读到fp 
   fh=fopen("shell.c","w+");
   // 创建一个可写的文件 
   fread(buff_system,1,127,fp);
   //将fp的数据流读到buff中 
   fwrite(buff_system,1,127,fh);
   //将buff的数据写入fh指向的文件中   
   pclose(fp); 
   fclose(fh);   
   //we get all pictures' mask, flag and frontier detection result image in a new folder.
   std::cout<<"证明接下来的模块也可以开启"<<1<<std::endl;
   /////////////////////////////////////////////////////////////////////////
   //**Dynamic judgement step
   
   //**output bgr, converted depth image
   /////////////////////////////////////////////////////////////////////////
   //**VO configuration with loop detection
   
   return 0;   
  
}























