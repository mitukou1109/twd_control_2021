#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

#include <twd_control_2021/MotorSpeedControllerFeedback.h>

class TwdRobotStatePublisher
{
public:

  TwdRobotStatePublisher()
  {
    ros::NodeHandle nh(""), pnh("~");

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    feedback_sub_ = nh.subscribe("twd_controller/feedback", 10,
                                 &TwdRobotStatePublisher::feedbackCB, this);
    cart_yaw_sub_ = nh.subscribe("cart_yaw", 10, &TwdRobotStatePublisher::cartYawCB, this);

    pnh.param("odom_frame", odom_frame_, std::string(""));
    pnh.param("robot_base_frame", robot_base_frame_, std::string(""));
    pnh.param("towing", is_towing_, true);
    pnh.param("cart_rev_joint", cart_rev_joint_, std::string(""));
    pnh.param("publish_period", publish_period_, 0.02);
    pnh.param("enable_odom_tf", enable_odom_tf_, true);

    timer_ = nh.createTimer(ros::Duration(publish_period_), &TwdRobotStatePublisher::publish, this);
    
    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = robot_base_frame_;
    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = robot_base_frame_;
    odom_tf_.transform.rotation.z = 1.0;

    if(is_towing_)
    {
      cart_rev_joint_state_.header.frame_id = robot_base_frame_;
      cart_rev_joint_state_.name.assign({cart_rev_joint_});
      cart_rev_joint_state_.position.resize(1);
    }
  }

  void publish(const ros::TimerEvent& e)
  {
    odom_tf_.header.stamp = ros::Time::now();
    odom_pub_.publish(odom_);
    if(enable_odom_tf_) tf_broadcaster_.sendTransform(odom_tf_);

    if(is_towing_)
    {
      cart_rev_joint_state_.header.stamp = ros::Time::now();
      joint_state_pub_.publish(cart_rev_joint_state_);
    }
  }

private:

  struct Pose2D
  {
    Pose2D() : x(0), y(0), theta(0) {}

    geometry_msgs::Quaternion quat()
    {
      tf2::Quaternion quat;
      quat.setRPY(0, 0, theta);
      return tf2::toMsg(quat);
    }

    void transform(double vel_x, double vel_theta, double dt)
    {
      x += vel_x*std::cos(theta)*dt;
      y += vel_x*std::sin(theta)*dt;
      theta += vel_theta*dt;
    }

    double x;
    double y;
    double theta;
  };

  void feedbackCB(const twd_control_2021::MotorSpeedControllerFeedback& feedback)
  {
    static ros::Time last_time;

    ros::Time current_time = ros::Time::now();
    robot_base_pose_.transform(feedback.msd_vel.linear.x, feedback.msd_vel.angular.z,
                               (current_time-last_time).toSec());

    odom_tf_.transform.translation.x = robot_base_pose_.x;
    odom_tf_.transform.translation.y = robot_base_pose_.y;
    odom_tf_.transform.translation.z = 0;
    odom_tf_.transform.rotation = robot_base_pose_.quat();

    odom_.twist.twist = feedback.msd_vel;
    odom_.pose.pose.position.x = robot_base_pose_.x;
    odom_.pose.pose.position.y = robot_base_pose_.y;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation = robot_base_pose_.quat();

    last_time = current_time;
  }

  void cartYawCB(const std_msgs::Float32 cart_yaw)
  {
    cart_rev_joint_state_.position.at(0) = cart_yaw.data;
  }

  ros::Timer timer_;

  ros::Publisher odom_pub_, joint_state_pub_;
  ros::Subscriber feedback_sub_, cart_yaw_sub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  nav_msgs::Odometry odom_;
  geometry_msgs::TransformStamped odom_tf_;

  Pose2D robot_base_pose_;

  sensor_msgs::JointState cart_rev_joint_state_;

  std::string odom_frame_, robot_base_frame_, cart_rev_joint_;

  double publish_period_;

  bool is_towing_, enable_odom_tf_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twd_robot_state_publisher");

  TwdRobotStatePublisher robot_state_publisher;

  ros::spin();

  return 0;
}