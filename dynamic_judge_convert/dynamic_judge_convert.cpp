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
#include <math.h>
#include <chrono>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
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
vector<cv::Vec3f> color_all;
vector<int> id_all;
vector<string> line_temp;

using namespace cv;
using namespace std;
void read_log_new(string file_path,vector<cv::Vec3f> &color_all_new,vector<int> &id_all_new)
{
        vector<string> line_temp_new;
        ifstream in(file_path);
	string filename;
	string line;
        string line_file;
	if(in) 
	{
	  while (getline (in, line_file)) 
	  // line中不包括每行的换行符
	  { 
	      
	      line_temp_new.push_back(line_file);
	  }
	}
	else 
	{
	    cout <<"no such file" << endl;
	}
	int j(0);
	cv::Vec3f temp;
        double b_0(0);
        double g_1(0);
        double r_2(0);
        char c_b[30];
	char c_g[30];
	char c_r[30];
	char c_id[30];
	int c_flag(0);
	int id(0);
	//I have read all information from the file_path;
	while(j<=line_temp_new.size()-1)
	{
	  if((j%2)==0)
	  {
	    int i(1);
	    line=line_temp_new[j];
            //extract and get a vec3b element
            c_flag=0;
	    while(line[i]!=',')
	    {
	       c_b[c_flag]=line[i];
	       c_flag++;
	       i++;
	    }
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
	    while(line[i]!=')')
	    {
	       c_r[c_flag]=line[i];
	       c_flag++;
	       i++;
	    }
	    c_flag=0;
	 
	    b_0=atof(c_b);
	    g_1=atof(c_g);
	    r_2=atof(c_r);
		   
            temp[0]=b_0;
	    temp[1]=g_1;
	    temp[2]=r_2;
		    
            color_all_new.push_back(temp);
	    //Added by Tianhao Wang
	    memset(c_b,'\0',sizeof(c_b));
	    memset(c_g,'\0',sizeof(c_g));
	    memset(c_r,'\0',sizeof(c_r));
	
	    //
	  }
	  else
	  {
	    line=line_temp_new[j];
	    int k(0);
	    int c_flag(0);
	    while(k<=line.size()-1)
	    {
	      c_id[c_flag]=line[k];
	      c_flag++;
	      k++;
	    }
	    id=atoi(c_id);
	    id_all_new.push_back(id);
	    memset(c_id,'\0',sizeof(c_id));
	  }
	  j++;
	}	  
}




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
		    while(line[i]!=')')
		    {
		      c_r[c_flag]=line[i];
		      c_flag++;
		      i++;
		    }
		    i++;
		    c_flag=0;
		    while(i<=line.size()-1)
		    {
		    c_id[c_flag]=line[i];
		    c_flag++;
		    i++;
		    };
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
		    cout<<temp<<endl;
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

float calDistance(float coor_x,float coor_y,float A,float B,float C)
{
  float temp(0);
  temp=fabs(coor_x*A+coor_y*B+C);
  return temp/(sqrt(A*A+B*B));
}

//Modified by Tianhao Wang, the last version deal with all the points in a image respectively
//In this recent version, I get out which objects is dynamic_judgement
//1:more than three dynamic points
//2:more than a specific percent, it requires calculating the total number of keypoints in an object 
//3:some special object like televison can be set a reference
//4:human being's object must be deleted

//input a number from keyboard to decide which folder during combination
int compare_color(cv::Vec3b a_color,cv::Vec3b b_color)
{
  //cout 完整的Vec3b值可以，但是单独输出一位，就会出错的。
  
  if((fabs(a_color[0]-b_color[0])<=40)&&(fabs(a_color[1]-b_color[1])<=40)&&(fabs(a_color[2]-b_color[2])<=40))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}


//there exists error when using "at" to visit mat(k,l) 
void dynamic_judgement_convert(string path_bgr,string path_mask_color,string path_depth_flip,vector<cv::Point2f> dynamic_judgement)
{
    // string rgb_file, depth_file, time_rgb, time_depth;
      list< cv::Point2f > keypoints;      
      // 因为要删除跟踪失败的点，使用list
      cv::Mat color, depth, last_color;
      cv::Mat mask_color;
      cv::Mat fundamental_matrix;
      
       int flag_count_dis=0;
       int total(0);
      //declare the mat ahead of time   
      //Modified by Tianhao Wang on 5.21
      //I must calculate how many valid image in this file;    
      for ( int index(0); index<9; index++ )
      {
	  //fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
	  
	  boost::format fmt( "%s%d.%s" );
	  boost::format fmt_temp( "%d.%s" );
	  
	  string path_file;
	  path_file=path_bgr;
	  path_file.append((fmt%"bgr"%(index)%"jpg").str());
	  color=cv::imread(path_file);
	  
	  path_file=path_mask_color;
	  path_file.append((fmt%"/mask_bgr"%(index)%"jpg").str());
	  mask_color=cv::imread(path_file,-1);
	  
	  path_file=path_mask_color;
	  path_file.append((fmt%"/log"%(index)%"txt").str());
	  vector<cv::Vec3f> color_all_new;
          vector<int> id_all_new;
          read_log_new(path_file,color_all_new,id_all_new);
	  
	  //in read log, the global vector will be cleared;
	  /*
	  //////////////////////////////////////////////////////
	  path_file=path_depth_flip;
	  path_file.append((fmt_temp%(index)%"png").str());
	  depth=cv::imread(path_file);
	  //////////////////////////////////////////////////////
	  */
	  
	  if (index == 0)
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
	  
	  if ( color.data==nullptr && depth.data==nullptr )
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
	  cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
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
	  int count_all_keypoints(0);
	  count_all_keypoints=keypoints.size();
	  
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
	  
	  //Modified by Tianhao Wang on 2019.4.7, this part is used to calculate the epilines
	  std::vector<cv::Vec<float,3>> epilines1,epilines2;
	  cv::computeCorrespondEpilines(prev_keypoints_correspond_vector,1,fundamental_matrix,epilines2);
	  cv::computeCorrespondEpilines(keypoints_vector,2,fundamental_matrix,epilines1);
	  //the integer(the second input) indicates which img the points set belongs to.
	  //Attention:epilines2, which is in the current frame is obtained according to the points set in previous frame. 
	  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  cv::RNG rng(time(0));
	  
	  for(uint i(0);i<keypoints_vector.size();i=i+10)
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
	  cv::Mat img_result=color.clone();
	  cout<<keypoints_vector.size()<<endl;
	  
	  flag_count_dis=0;
	  //this value flag_count_dis is used to calculate how many points' distance is shorter than the current one 
	  for(uint i(0);i<keypoints_vector.size();i++)
	  {
	    distance_ce[i]=calDistance(keypoints_vector[i].x,keypoints_vector[i].y,epilines2[i][0],epilines2[i][1],epilines2[i][2]);
	    //std::cout<<"距对应的极线距离是"<<distance_ce[i]<<std::endl;
	  }
	  //calculate the maxium 
	  float temp_max(0);
	  for(int i(0);i<=keypoints_vector.size()-1;i++)
	  {
	    if(distance_ce[i]>=temp_max)
	    {
	      temp_max=distance_ce[i];
	    }
	  }
	  ///////////////////////////////////
	  for(uint i(0);i<keypoints_vector.size();i++)
	  {
	    for(uint j(0);j<keypoints_vector.size();j++)
	    {
	      if(distance_ce[i]>distance_ce[j])
	      {
		flag_count_dis++;
	      }
	    }
	    if(flag_count_dis>=(0.97)*keypoints_vector.size()&&distance_ce[i]>70)
	    {
	      cv::circle(img_result, keypoints_vector[i], 10, cv::Scalar(240, 0, 0), 1);
	      dynamic_judgement.push_back(keypoints_vector[i]);
	    }
	    flag_count_dis=0;
	  }
	  
	  cv::destroyAllWindows();
	  cv::imshow("final_result",img_result);
	  cv::waitKey(3000);
	  //sort part and get the threshold(how to get the threshold?)
	  //save these result image for paper in a subdirectory file
	  string path_file_save_1;
	  path_file_save_1=path_bgr;
	  string path_file_save_2;
	  path_file_save_2=path_bgr;
	  string path_file_save_3;
	  path_file_save_3=path_bgr;
	  path_file_save_1.append((fmt%"result/bgr"%(index)%"jpg").str());
	  path_file_save_2.append((fmt%"result/bgr_original"%(index)%"jpg").str());
	  path_file_save_3.append((fmt%"result/bgr_final"%(index)%"jpg").str());
	  cv::imwrite(path_file_save_1,img_show);
	  cv::imwrite(path_file_save_2,img_show_original);
	  cv::imwrite(path_file_save_3,img_result);
	  
	  /////////////////////////////////////////////////////////////////////////////
	  //convert depth image step
	  /*
	  cout<<"id_all_new size is "<<id_all_new.size()<<endl;
	  int count_color[id_all_new.size()];
	  for(int i(0);i<=id_all_new.size()-1;i++)
	  {
	    count_color[i]=0;
	  }
	  ////////////////////////////////////////////////////////////////////////////
	  vector <cv::Vec3b> color_converted;
	  for(int i(0);i<=color_all_new.size()-1;i++)
	  {
	    cv::Vec3b color_temp;
	    color_temp[0]=255*color_all_new[i][0];
	    color_temp[1]=255*color_all_new[i][1];
	    color_temp[2]=255*color_all_new[i][2];
	    color_converted.push_back(color_temp);
	  }
	  for(int i(0);i<=color_all_new.size()-1;i++)
	  {
	    cout<<"color_mask"<<color_all_new[i]<<endl;
	  }
	  for(int i(0);i<=color_converted.size()-1;i++)
	  {
	    cout<<"color_converted"<<color_converted[i]<<endl;
	  }
	  */
	  
	  cv::Mat new_bgr;
	  new_bgr=color.clone();
          boost::format fmt_temp_new("%s%d%s%d.%s");
	  
	
	  cout<<"size of dynamic_judgement "<<dynamic_judgement.size()<<endl;
	  if(dynamic_judgement.size()==0)
	  {
	    cv::imshow("converted_bgr",new_bgr);
	    cv::waitKey(3000);
	    cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/bgr/converted_bgr"%(index)%"jpg").str(),new_bgr);
	    dynamic_judgement.clear();
	    last_color = color;
	    continue;
	  }
	    
	  for(int i(0);i<=color_all_new.size()-1;i++)
	  {
	      cv::Vec3b color_temp;
	      color_temp[0]=255;
	      color_temp[1]=255;
	      color_temp[2]=255;
	      int count_now(0);
	      cv::Mat temp_reference;
	      temp_reference=cv::imread((fmt_temp_new%"/home/wth/design/Myslam/dataset/mask_bgr/mask_bgr"%(index)%"_"%(i)%"jpg").str(),-1);
	      cout<<"***************"<<endl;
	      
	      for(int j(0);j<=dynamic_judgement.size()-1;j++)
	      {
		  if(!compare_color(temp_reference.at<cv::Vec3b>(dynamic_judgement[j].x,dynamic_judgement[j].y),color_temp))
		  {
		    count_now++;
		  }
	      }
	      if(count_now>=(0.11)*(dynamic_judgement.size()))
	      {
		for(int u(0);u<temp_reference.rows;u++)
		{
		  for(int v(0);v<temp_reference.cols;v++)
		    {
		      if(!compare_color(temp_reference.at<cv::Vec3b>(u,v),color_temp))
		      {
			new_bgr.at<cv::Vec3b>(u,v)=color_temp;
		      }
		    }
		}
	      }
	  }
//
//	  
//	  
//Reference: there exists a pity that I failed to use color to divide different objects
//It's because that using 'at' function to visit
//
//
//


	  /*
          ////////////////////////////////////////////////////////////////////////////
	 
	  ////////////////////////////////////////////////////////////////////////////
	  for(int i(0);i<=dynamic_judgement.size()-1;i++)
	  {
	    for(int j(0);j<=color_all_new.size()-1;j++)
	    {
	      ////////////////////////////////////////////////////////
	      if(index==5)
	      {
	        cout<<"mask color is "<<mask_color.at<cv::Vec3b>(dynamic_judgement[i].x,dynamic_judgement[i].y)<<endl;
		cout<<"color_converted is "<<color_converted[j]<<endl;
	      }
	      ////////////////////////////////////////////////////////
	      if(compare_color(mask_color.at<cv::Vec3b>(dynamic_judgement[i].x,dynamic_judgement[i].y),color_converted[j])==1)
	      {
		count_color[j]=count_color[j]+1;
		break;
	      }
	    }
	  }

	  for(int i(0);i<=color_converted.size()-1;i++)
	  {
	    cout<<"count_color"<<count_color[i]<<endl;
	  }
	  ////////////////////////////////////////////////////////////////////////////
	  cv::Mat new_bgr;
	  new_bgr=color.clone();
	  
	  for(int u(0);u<new_bgr.rows;u++)
	   {
             for(int v(0);v<new_bgr.cols;v++)
	      {
		for(int i(0);i<=color_all_new.size()-1;i++)
		{
		  if(compare_color(mask_color.at<cv::Vec3b>(u,v),color_converted[i])&&count_color[i]>=3)
	          {
		   new_bgr.ptr<cv::Vec3b>(u)[v]=(255,255,255);
		  }
		}
	      }
	   }
	   */
	  cv::imshow("converted_bgr",new_bgr);
	  cv::waitKey(3000);
	  cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/bgr/converted_bgr"%(index)%"jpg").str(),new_bgr);
	  dynamic_judgement.clear();
	  last_color = color;
  }
  cout<<"总数"<<total<<endl;
}

cv::Affine3f vis_pose(float x,float y,float z)
{
   //Euler angel 10,10,10
   cv::Mat R_mat_X=(cv::Mat_<float>(3,3) << 1,0,0,0,cos(CV_PI*(x/180)),-sin(CV_PI*(x/180)),0,sin(CV_PI*(x/180)),cos(CV_PI*(x/180))); 
   cv::Mat R_mat_Y=(cv::Mat_<float>(3,3) << cos(CV_PI*(y/180)),0,sin(CV_PI*(y/180)),0,1,0,-sin(CV_PI*(y/180)),0,cos(CV_PI*(y/180))); 
   cv::Mat R_mat_Z=(cv::Mat_<float>(3,3) << cos(CV_PI*(z/180)),-sin(CV_PI*(z/180)),0,sin(CV_PI*(z/180)),cos(CV_PI*(z/180)),0,0,0,1); 
   cv::Mat final_rotation;
   final_rotation=R_mat_X*R_mat_Y*R_mat_Z;
   cv::Affine3f pose(final_rotation, cv::Vec3f(0, 0, 0));
   return pose;
}

cv::Affine3f vis_pose_initial(float x,float y,float z)
{
   //Euler angel 10,10,10
   cv::Mat R_mat_X=(cv::Mat_<float>(3,3) << 1,0,0,0,cos(CV_PI*(x/180)),-sin(CV_PI*(x/180)),0,sin(CV_PI*(x/180)),cos(CV_PI*(x/180))); 
   cv::Mat R_mat_Y=(cv::Mat_<float>(3,3) << cos(CV_PI*(y/180)),0,sin(CV_PI*(y/180)),0,1,0,-sin(CV_PI*(y/180)),0,cos(CV_PI*(y/180))); 
   cv::Mat R_mat_Z=(cv::Mat_<float>(3,3) << cos(CV_PI*(z/180)),-sin(CV_PI*(z/180)),0,sin(CV_PI*(z/180)),cos(CV_PI*(z/180)),0,0,0,1); 
   cv::Mat final_rotation;
   final_rotation=R_mat_X*R_mat_Y*R_mat_Z;
   cv::Affine3f pose(final_rotation, cv::Vec3f(1, 0, 0));
   return pose;
}



void test_imu_estimation()
{
    int x(10);
    int y(0);
    int z(0);
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
    
    Affine3f initial;
    Affine3f initial_with_rvec=vis_pose_initial(0,0,0);
    initial=initial_with_rvec;
    Affine3f change=vis_pose(10,0,0);
    Affine3f pose;
    //位姿显示
    vis.setWidgetPose ( "Camera",initial );
    vis.spinOnce ( 300, false );
    
    while(1)
    {
       vis.updateWidgetPose("Camera",change);  
       vis.spinOnce ( 300, false );
    }
}




void simple_convert_depth(string file)//6-13
{
  

  for(int flag(4);flag<=6;flag++)
  { 
    cv::Mat mask_trick;
    cv::Mat bgr_trick;
    cv::Mat depth_trick;
    cv::Mat flipmImageDepth;
    boost::format fmt( "%s%d.%s" );
    string path_file=file;
    path_file.append((fmt%"/log"%(flag)%"txt").str());
    mask_trick=cv::imread((fmt%"/home/wth/design/Myslam/dataset/mask_bgr_1/unkeyframe/t/mask_bgr"%(flag)%"jpg").str(),-1);
    depth_trick=cv::imread((fmt%"/home/wth/design/Myslam/dataset/depth_flip_1/unkeyframe/t/depth_flip"%(flag)%"png").str());
    cout<<mask_trick.size()<<endl;
    cout<<depth_trick.size()<<endl;
  
      //compare and convert depth
      
      for(int m(0);m<mask_trick.rows;m++)
	{
	  for(int n(0);n<mask_trick.cols;n++)
	  {
	    if((mask_trick.at<cv::Vec3b>(m,n)[0]<=230)||(mask_trick.at<cv::Vec3b>(m,n)[1]<=230)||(mask_trick.at<cv::Vec3b>(m,n)[2]<=230))
	   {
	      
	      depth_trick.ptr<int>(m)[n-40]=10000;
	   }
	  }
	}
	cout<<"******"<<flag<<endl;
      cv::imwrite((fmt%"/home/wth/design/Myslam/dataset/depth_flip_1/unkeyframe/t/trick/depth_flip"%(flag)%"png").str(),depth_trick);
    }
}

void color_check(vector<cv::Vec3b> &color_all_check,cv::Mat a)
{
  cv::Vec3b color_reference;
  cv::Vec3b color_temp;
  color_reference[0]=255;
  color_reference[1]=255;
  color_reference[2]=255;
  cout<<"the first size of color_all_check is "<<color_all_check.size()<<endl;
  for(int u(0);u<a.rows;u++)
  {
    for(int v(0);v<a.cols;v++)
    {
      if(color_all_check.size()==0)
      {
	if(compare_color(a.at<cv::Vec3b>(u,v),color_reference)==0)
	{
	color_temp=a.at<cv::Vec3b>(u,v);
	cout<<"the first color is "<<color_temp<<endl;
	color_all_check.push_back(color_temp);
	}
      }
      else
      {
	int flag(0);
	for(int m(0);m<=color_all_check.size()-1;m++)
	{
	  if((!compare_color(a.at<cv::Vec3b>(u,v),color_all_check[m]))&&(!compare_color(a.at<cv::Vec3b>(u,v),color_reference)))
	  {
	    flag++;
	  }
	}
	if (flag==color_all_check.size())
	{
	  color_temp=a.at<cv::Vec3b>(u,v);
	  color_all_check.push_back(color_temp);
	}
      }
    }
  }
}
  
int main ( int argc, char** argv )
{ 
   vector<cv::Point2f> dynamic_judgement;
   dynamic_judgement_convert("/home/wth/design/Myslam/dataset/bgr/","/home/wth/design/Myslam/dataset/mask_bgr","/home/wth/design/Myslam/dataset/depth/",dynamic_judgement);
   //***********************************************
   //test_imu_estimation();
   //***********************************************
  
  
   /*
   vector<cv::Vec3b> color_all_check;
   cv::Mat a;
   a=imread("/home/wth/design/Myslam/dataset/mask_bgr_1/mask_bgr8.jpg");
   color_check(color_all_check,a);
   cout<<color_all_check.size();
   for(int i(0);i<=color_all_check.size()-1;i++)
   {
     cout<<color_all_check[i]<<endl;
   }
   */
   
   return 0;
}












