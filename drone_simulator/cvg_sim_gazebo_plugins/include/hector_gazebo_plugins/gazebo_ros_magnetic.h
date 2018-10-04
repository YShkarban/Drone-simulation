
#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_MAGNETIC_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_MAGNETIC_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{

class GazeboRosMagnetic : public ModelPlugin
{
public:
  GazeboRosMagnetic();
  virtual ~GazeboRosMagnetic();

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
  ros::Publisher publisher_;

  geometry_msgs::Vector3Stamped magnetic_field_;
  gazebo::math::Vector3 magnetic_field_world_;

  std::string namespace_;
  std::string topic_;
  std::string link_name_;
  std::string frame_id_;

  double magnitude_;
  double reference_heading_;
  double declination_;
  double inclination_;

  SensorModel3 sensor_model_;

  /// \brief save last_time
  common::Time last_time;
  common::Time update_period;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif 