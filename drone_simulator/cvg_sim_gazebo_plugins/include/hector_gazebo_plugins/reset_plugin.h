
#ifndef HECTOR_GAZEBO_PLUGINS_RESET_PLUGIN_H
#define HECTOR_GAZEBO_PLUGINS_RESET_PLUGIN_H

#include "gazebo/common/Plugin.hh"

#include <ros/ros.h>

namespace gazebo
{

class GazeboResetPlugin : public ModelPlugin
{
public:
  GazeboResetPlugin();
  virtual ~GazeboResetPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

private:
  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;
};

} // namespace gazebo

#endif 