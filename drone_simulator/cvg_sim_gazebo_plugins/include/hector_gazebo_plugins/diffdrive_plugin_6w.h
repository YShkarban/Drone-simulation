
#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"
//#include "gazebo/physics/physics.h"
//#include "transport/TransportTypes.hh"
//#include "msgs/MessageTypes.hh"
//#include "gazebo/common/Time.hh"
//#include "gazebo/common/Events.hh"

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class DiffDrivePlugin6W : public ModelPlugin
{

public:
  DiffDrivePlugin6W();
  virtual ~DiffDrivePlugin6W();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  void publish_odometry();
  void GetPositionCmd();

  physics::LinkPtr link;
  physics::WorldPtr world;
  physics::JointPtr joints[6];

  float wheelSep;
  float wheelDiam;
  float torque;
  float wheelSpeed[2];

  // Simulation time of the last update
  common::Time prevUpdateTime;

  bool enableMotors;
  float odomPose[3];
  float odomVel[3];

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string robotNamespace;
  std::string topicName;
  std::string linkName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  float x_;
  float rot_;
  bool alive_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif

