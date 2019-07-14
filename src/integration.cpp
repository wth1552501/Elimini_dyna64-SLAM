#include <iostream>
#include <cmath>
#include "stdio.h"
#include "stdlib.h"
#include "myslam/integration.h"
using namespace myslam;
using namespace std;

IMU_integration::IMU_integration()
{
  t=0.01;
  pre_acc=0;
  pre_velocity=0;
  flag_immobile=0;
}

IMU_integration::~IMU_integration()
{
  
}

double IMU_integration::calintegral(double acc)
{
   // as for the contimuous moving of imu
  if((fabs(acc-pre_acc))>0.03)
  {
    flag_immobile=1;//represent imu moves
    if(acc>pre_acc)
    {
      displacement=pre_velocity*t+0.5*(acc-0.015)*t*t;
      pre_velocity=pre_velocity+(acc-0.015)*t;
      pre_acc=acc;
    }// speed up period
    else
    {
      displacement=pre_velocity*t+0.5*(acc-0.015)*t*t;
      pre_velocity=pre_velocity+(acc-0.015)*t;
      pre_acc=acc;
    }//speed down period 
  }
  //Under this condition, I consider imu as immobile.
  else
  {
    pre_velocity=0;
  }
}

//
