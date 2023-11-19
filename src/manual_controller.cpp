#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <twd_control_2021/MotorSpeedControllerConfig.h>
#include <twd_control_2021/MotorSpeedControllerParam.h>

class ManualController
{
public:

  using MotorSpeedControllerConfigServer =
    dynamic_reconfigure::Server<twd_control_2021::MotorSpeedControllerConfig>;

  ManualController() :
    is_joy_locked_(true)
  {
    ros::NodeHandle nh(""), pnh("~");

    cmd_vel_pub_ = pnh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    param_pub_ = pnh.advertise<twd_control_2021::MotorSpeedControllerParam>("param", 1);
    joy_sub_ = nh.subscribe("joy", 1, &ManualController::joyCB, this);
    path_following_status_sub_ =
      nh.subscribe("path_following_controller/busy", 1,
                   &ManualController::pathFollowingStatusCB, this);

    pnh.param("publish_rate", publish_rate_, 10.0);

    timer_ = nh.createTimer(ros::Rate(publish_rate_),
                            &ManualController::publishCmdVel, this);
    
    config_server_.setCallback(boost::bind(&ManualController::reconfigureCB, this, _1, _2));
  }

private:

  void joyCB(const sensor_msgs::Joy& joy)
  {
    float base_vel_x = (joy.buttons[4] == 1) ? 0.1 :
                       (joy.buttons[5] == 1) ? 0.4 : 0.2;
    float base_vel_theta = (joy.buttons[4] == 1) ? M_PI/6 :
                           (joy.buttons[5] == 1) ? M_PI/2 : M_PI/3;

    if(joy.buttons[9] == 1)
    {
      is_joy_locked_ = true;
    }
    else if(joy.buttons[8] == 1)
    {
      is_joy_locked_ = false;
    }

    if(is_joy_locked_)
    {
      cmd_vel_.linear.x = cmd_vel_.angular.z = 0;
    }
    else
    {
      if(std::abs(joy.axes[7]) > 0 && joy.axes[3] == 0.0)
      {
        cmd_vel_.linear.x = std::copysign(base_vel_x, joy.axes[7]);
        cmd_vel_.angular.z = 0;
      }
      else if(std::abs(joy.axes[3]) > 0 && joy.axes[7] == 0.0)
      {
        cmd_vel_.linear.x = 0;
        cmd_vel_.angular.z = std::copysign(base_vel_theta, joy.axes[3]);
      }
      else
      {
        cmd_vel_.linear.x = joy.axes[1]*base_vel_x;
        cmd_vel_.angular.z = joy.axes[0]*base_vel_theta;
      }
    }
  }

  void pathFollowingStatusCB(
    const std_msgs::Bool busy)
  {
    is_path_following_active_ = busy.data;
  }

  void reconfigureCB(twd_control_2021::MotorSpeedControllerConfig& config, 
                     uint32_t level)
  {
    twd_control_2021::MotorSpeedControllerParam param;

    param.rpm_to_voltage_gain[0] = config.rpm_to_voltage_gain_l;
    param.rpm_to_voltage_gain[1] = config.rpm_to_voltage_gain_r;
    param.p_gain = config.p_gain;
    param.i_gain = config.i_gain;
    param.d_gain = config.d_gain;
    param.min_voltage = config.min_voltage;
    param.max_voltage = config.max_voltage;

    param_pub_.publish(param);
  }

  void publishCmdVel(const ros::TimerEvent& e)
  {
    if(is_path_following_active_) return;
    
    if(is_joy_locked_)
    {
      cmd_vel_.linear.x = cmd_vel_.angular.z = 0;
    }

    cmd_vel_pub_.publish(cmd_vel_);
  }
  
  ros::Publisher cmd_vel_pub_, param_pub_;
  ros::Subscriber joy_sub_, path_following_status_sub_;

  ros::Timer timer_;

  MotorSpeedControllerConfigServer config_server_;

  geometry_msgs::Twist cmd_vel_;

  double publish_rate_;

  bool is_joy_locked_, is_path_following_active_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_controller");

  ManualController manual_controller;

  ros::spin();

  return 0;
}