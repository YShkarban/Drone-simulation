
#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{

class GazeboRosSonar : public SensorPlugin
{
public:
  GazeboRosSonar();
  virtual ~GazeboRosSonar();

protected:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  sensors::RaySensorPtr sensor_;

  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;

  sensor_msgs::Range range_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;

  SensorModel sensor_model_;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif 