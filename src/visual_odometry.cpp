/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{

  VisualOdometry::VisualOdometry() :
	  state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
  {
	  num_of_features_    = Config::get<int> ( "number_of_features" );
	  scale_factor_       = Config::get<double> ( "scale_factor" );
	  level_pyramid_      = Config::get<int> ( "level_pyramid" );
	  match_ratio_        = Config::get<float> ( "match_ratio" );
	  max_num_lost_       = Config::get<float> ( "max_num_lost" );
	  min_inliers_        = Config::get<int> ( "min_inliers" );
	  key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
	  key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
	  map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
	  orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
  }

  VisualOdometry::~VisualOdometry()
  {

  }

  bool VisualOdometry::addFrame ( Frame::Ptr frame )
  {
	  switch ( state_ )
	  {
		case INITIALIZING:
		{
		  state_ = OK;
		  curr_ = ref_ = frame;
		  // extract features from first frame and add them into map
		  extractKeyPoints();
		  computeDescriptors();
		  addKeyFrame();      // the first frame is a key-frame
		  break;
		}
		case OK:
		{
		  //当前帧为参考帧
		  curr_ = frame;
		  //当前位姿为参考帧位姿
		  curr_->T_c_w_ = ref_->T_c_w_;
		  //计算角点
		  cout<<"Extract KeyPoint Now!!!" <<endl;
		  extractKeyPoints();
		  //计算描述子
		  cout<<"Compute Descriptors Now!!!" <<endl;
		  computeDescriptors();
		  //进行特征匹配
		  cout<<"Feature Matching Now!!!" <<endl;
		  featureMatching();
		  
		  //如果特征匹配失败
		  if(match_state == 0)
		  {
			  num_lost_++;
			  if ( num_lost_ > max_num_lost_ )
			  {
			  cout<<"Feature Matching LOST!!!" <<endl;
			  state_ = LOST;
			  }
			  error_frame = 1;
			  return false;
		  }
		  
		  //PnP位姿估计
		  cout<<"Pose Estimate PnP Now!!!" <<endl;
		  poseEstimationPnP();

		  if(PnP_state == 0)
		  {
			  num_lost_++;
			  if ( num_lost_ > max_num_lost_ )
			  {
			  cout<<"Pose Estimate PnP LOST!!!" <<endl;
			  state_ = LOST;
			  }
			  error_frame = 1;
			  return false;
		  }

		  if ( checkEstimatedPose() == true ) // a good estimation
		  {
			  curr_->T_c_w_ = T_c_w_estimated_;
			  /////////////////////////////////////////////
			  curr_->frame_rvec=my_rvec;
			  curr_->frame_tvec=my_tvec;
			  /////////////////////////////////////////////
			  optimizeMap();
			  num_lost_ = 0;
			  if ( checkKeyFrame() == true ) // is a key-frame
			  {
				addKeyFrame();
			  }else
			  {
			  error_frame = 1;
			  }
		  }
		  else // bad estimation due to various reasons
		  {
			  num_lost_++;
			  cout<<"Check Estimate Pose LOST!!!" <<endl;
			  if ( num_lost_ > max_num_lost_ )
			  {
				state_ = LOST;
			  }
			  error_frame = 1;
			  return false;
		  }
		  error_frame = 0; //位姿估计正确
		  break;
		}
		case LOST:
		{
		  cout<<"vo has lost."<<endl;
		  error_frame = 1;
		  break;
		}
	  }

	  return true;
  }

  //检测角点
  void VisualOdometry::extractKeyPoints()
  {
	  //计时
	  boost::timer timer;
	  orb_->detect ( curr_->color_, keypoints_curr_ );
	  cout<<"orb detect: "<<keypoints_curr_.size() <<endl;
	  cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
  }

  //根据角点计算BRIEF描述子
  void VisualOdometry::computeDescriptors()
  {
	  //计时
	  boost::timer timer;
	  orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
	  cout<<"orb compute: "<<descriptors_curr_.size() <<endl;
	  cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
  }


  //对两幅图像中的BRIEF描述子进行特征匹配
  void VisualOdometry::featureMatching()
  {
	  //计时
	  boost::timer timer;
	  
	  vector<cv::DMatch> matches;

	  cout<<"Select the candidates in map" <<endl;
	  // select the candidates in map 
	  Mat desp_map;
	  vector<MapPoint::Ptr> candidate;

	  //搜索地图上所有路标点
	  for ( auto& allpoints: map_->map_points_ )
	  {
		  MapPoint::Ptr& p = allpoints.second;
		  //如果路标点在当前视野内
		  if ( curr_->isInFrame(p->pos_) )
		  {
			  //将路标点加入匹配的特征点
			  p->visible_times_++;
			  candidate.push_back( p );
			  desp_map.push_back( p->descriptor_ );
		  }
	  }
	  
	  //快速近似最近邻(FLANN)算法进行特征匹配
	  matcher_flann_.match ( desp_map, descriptors_curr_, matches );

	  cout<<"Select the best matches" <<endl;
	  //判断是否匹配成功
	  if (matches.size() >=10)
	  {   //特征匹配点数足够多
		  match_state = 1;
		  // 描述子的最小值
		  float min_dis = std::min_element (
							  matches.begin(), matches.end(),
							  [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
		  {
			  return m1.distance < m2.distance;
		  } )->distance;

		  cout<<"Clear" <<endl;
		  match_3dpts_.clear();
		  match_2dkp_index_.clear();

		  cout<<"Push back the  matches" <<endl;
		  for ( cv::DMatch& m : matches )
		  {
			  if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
			  {
				  match_3dpts_.push_back( candidate[m.queryIdx] );
				  match_2dkp_index_.push_back( m.trainIdx );
			  }
		  }
		  cout<<"good matches: "<<match_3dpts_.size() <<endl;
		  cout<<"match cost time: "<<timer.elapsed() <<endl;
	  }else
	  {
		  //特征匹配失败
		  match_state = 0;
		  cout<<"less matches: "<<matches.size() <<endl;
	  }
  }

  void VisualOdometry::poseEstimationPnP()
  {
	  // construct the 3d 2d observations
	  vector<cv::Point3f> pts3d;
	  vector<cv::Point2f> pts2d;

	   
	  //存入相邻帧匹配的特征点索引
	  for ( int index:match_2dkp_index_ )
	  {
		  pts2d.push_back ( keypoints_curr_[index].pt );
	  }
	  cout<<"Feature Put 2d!!!    pts2d=" <<pts2d.size()<<endl;
	  
	  //存入相邻帧匹配的特征点
	  for ( MapPoint::Ptr pt:match_3dpts_ )
	  {
		  pts3d.push_back( pt->getPositionCV() );
	  }
	  cout<<"Feature Put 3d!!!    pts3d=" <<pts3d.size()<<endl;
	  
	  //内参矩阵
	  Mat K = ( cv::Mat_<double> ( 3,3 ) <<
				ref_->camera_->fx_, 0, ref_->camera_->cx_,
				0, ref_->camera_->fy_, ref_->camera_->cy_,
				0,0,1
			  );
	  Mat rvec, tvec, inliers;

	  cout<<"solve Pnp!!!" <<endl;

	  if((pts3d.size() >=4) && (pts2d.size() >= 4))
	  {
		  //PnP成
		  PnP_state = 1;
		  // 调用 OpenCV 的 PnP 求解,可选择 EPNP , DLS 等方法
		  cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
		  //Add reference by Tianhao Wang on 5.19
		  //////////////////////////////////////////////////////
		  my_rvec=rvec;
		  my_tvec=tvec;
		  //////////////////////////////////////////////////////
		  //inliers is output noarray
		  num_inliers_ = inliers.rows;
		  cout<<"pnp inliers: "<<num_inliers_<<endl;

		  //得到估计的位姿变换矩阵
		  T_c_w_estimated_ = SE3 (
								SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
								Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
							);

		  //使用BA优化位姿估计
		  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
		  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
		  Block* solver_ptr = new Block ( linearSolver );
		  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
		  g2o::SparseOptimizer optimizer;
		  optimizer.setAlgorithm ( solver );

		  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
		  pose->setId ( 0 );
		  pose->setEstimate ( g2o::SE3Quat (
			  T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
		  ));
		  optimizer.addVertex ( pose );

		  // edges
		  for ( int i=0; i<inliers.rows; i++ )
		  {
			  int index = inliers.at<int> ( i,0 );
			  // 3D -> 2D projection
			  EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
			  edge->setId ( i );
			  edge->setVertex ( 0, pose );
			  edge->camera_ = curr_->camera_.get();
			  edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
			  edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
			  edge->setInformation ( Eigen::Matrix2d::Identity() );
			  optimizer.addEdge ( edge );
			  // set the inlier map points 
			  match_3dpts_[index]->matched_times_++;
		  }

		  optimizer.initializeOptimization();
		  optimizer.optimize ( 10 );

		  T_c_w_estimated_ = SE3 (
			  pose->estimate().rotation(),
			  pose->estimate().translation()
		  );
		  //得到优化后的位姿变换矩阵
		  cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
	  }else
	  {
		  //PnP失败
		  PnP_state = 0;
		  cout<<"PnP求解失败" <<endl;  
		  cout<<"pts3d: "<<pts3d.size() <<endl;
		  cout<<"pts2d: "<<pts2d.size() <<endl; 
	  }
  }

  bool VisualOdometry::checkEstimatedPose()
  {
	  // check if the estimated pose is good
	  if ( num_inliers_ < min_inliers_ )
	  {
		  cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
		  return false;
	  }
	  // if the motion is too large, it is probably wrong
	  SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
	  Sophus::Vector6d d = T_r_c.log();
	  cout<<"motion: "<<d.norm() <<endl;
	  if ( d.norm() > 3.0 )
	  {
		  cout<<"reject because motion is too large: "<<d.norm() <<endl;
		  return false;
	  }
	  return true;
  }

  bool VisualOdometry::checkKeyFrame()
  {
	  SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
	  Sophus::Vector6d d = T_r_c.log();
	  Vector3d trans = d.head<3>();
	  Vector3d rot = d.tail<3>();
	  if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
		  return true;
	  return false;
  }

  void VisualOdometry::addKeyFrame()
  {
	  if ( map_->keyframes_.empty() )
	  {
		  // 将匹配到的角点加入地图
		  for ( size_t i=0; i<keypoints_curr_.size(); i++ )
		  {
			  double d = curr_->findDepth ( keypoints_curr_[i] );
			  //检测深度
			  if ( d <=0 ) 
				  continue;
			  //转为世界坐标
			  Vector3d p_world = ref_->camera_->pixel2world (
				  Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
			  );
			  Vector3d n = p_world - ref_->getCamCenter();
			  n.normalize();
			  MapPoint::Ptr map_point = MapPoint::createMapPoint(
				  p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
			  );
			  map_->insertMapPoint( map_point );
		  }
	  }
	  
	  map_->insertKeyFrame ( curr_ );
	  ref_ = curr_;
  }

  void VisualOdometry::addMapPoints()
  {
	  // add the new map points into map
	  vector<bool> matched(keypoints_curr_.size(), false); 
	  for ( int index:match_2dkp_index_ )
		  matched[index] = true;
	  for ( int i=0; i<keypoints_curr_.size(); i++ )
	  {
		  if ( matched[i] == true )   
			  continue;
		  double d = ref_->findDepth ( keypoints_curr_[i] );
		  if ( d<0 )  
			  continue;
		  Vector3d p_world = ref_->camera_->pixel2world (
			  Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
			  curr_->T_c_w_, d
		  );
		  Vector3d n = p_world - ref_->getCamCenter();
		  n.normalize();
		  MapPoint::Ptr map_point = MapPoint::createMapPoint(
			  p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
		  );
		  map_->insertMapPoint( map_point );
	  }
  }

  void VisualOdometry::optimizeMap()
  {
	  // remove the hardly seen and no visible points 
	  for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
	  {
		  if ( !curr_->isInFrame(iter->second->pos_) )
		  {
			  //特征点不在当前帧内就删除点
			  iter = map_->map_points_.erase(iter);
			  continue;
		  }

		  float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
		  if ( match_ratio < map_point_erase_ratio_ )
		  {
			  iter = map_->map_points_.erase(iter);
			  continue;
		  }
		  
		  double angle = getViewAngle( curr_, iter->second );
		  if ( angle > M_PI/6. )
		  {
			  iter = map_->map_points_.erase(iter);
			  continue;
		  }
		  if ( iter->second->good_ == false )
		  {
			  // TODO try triangulate this map point 
		  }
		  iter++;
	  }
	  
	  if ( match_2dkp_index_.size()<500 )
	  {
		addMapPoints();
	  }
		  
	  if ( map_->map_points_.size() > 1000 )  
	  {
		  // TODO map is too large, remove some one 
		  map_point_erase_ratio_ += 0.05;
	  }
	  else {
		  map_point_erase_ratio_ = 0.1;
	  }
	  cout<<"map points: "<<map_->map_points_.size()<<endl;
  }

  double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
  {
	  Vector3d n = point->pos_ - frame->getCamCenter();
	  n.normalize();
	  return acos( n.transpose()*point->norm_ );
  }

}
