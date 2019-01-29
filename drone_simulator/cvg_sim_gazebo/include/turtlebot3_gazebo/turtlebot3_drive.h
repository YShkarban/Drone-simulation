

#ifndef DRONE_DRIVE_H_
#define DRONE_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

//Scan data sides
#define CENTER 0
#define LEFT   1
#define RIGHT  2

//Drone speed
#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5


class DroneDrive
{
 public:
  DroneDrive();
  ~DroneDrive();
  bool init();
  bool controlLoop();
  void takeoffCommand();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;


  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher takeoff_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber state_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  double drone_pose_;
  double prev_drone_pose_;

  // Functions
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void stateMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  
};
#endif // DRONE_DRIVE_H_
