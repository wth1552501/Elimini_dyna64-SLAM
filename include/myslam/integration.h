#ifndef _INTEGRATION_
#define _INTEGRATION_
//Modified by Tianhao Wang on 2019.5.3
//Tianhao Wang wants to calculate the integral of velocity of the camera according to 
//the accelemetor output of 9d0f IMU.
//This is a simple integration without a filter's correction

namespace myslam
{
  class IMU_integration
  {
    private:
      double t;
      double pre_acc;
      double pre_velocity;
    public:
      double displacement;
      int flag_immobile;//simply judge whether imu moves on some x or y axis
      double calintegral(double accc);
      IMU_integration();
      ~IMU_integration();
  };
}
#endif
