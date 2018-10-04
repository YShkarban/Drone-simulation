
#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_BARO_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_BARO_H

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"

#include <ros/ros.h>
#ifdef USE_MAV_MSGS
  #include <mav_msgs/Height.h>
#else
  #include <geometry_msgs/PointStamped.h>
#endif
#include <cvg_sim_msgs/Altimeter.h>

#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{

class GazeboRosBaro : public ModelPlugin
{
public:
  GazeboRosBaro();
  virtual ~GazeboRosBaro();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher height_publisher_;
  ros::Publisher altimeter_publisher_;

#ifdef USE_MAV_MSGS
  mav_msgs::Height height_;
#else
  geometry_msgs::PointStamped height_;
#endif
  cvg_sim_msgs::Altimeter altimeter_;

  std::string namespace_;
  std::string height_topic_;
  std::string altimeter_topic_;
  std::string link_name_;
  std::string frame_id_;

  double elevation_;
  double qnh_;

  SensorModel sensor_model_;

  /// \brief save last_time
  common::Time last_time;
  common::Time update_period;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_BARO_H
