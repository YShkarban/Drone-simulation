
#include <hector_gazebo_plugins/reset_plugin.h>
#include "gazebo/common/Events.hh"

#include <std_msgs/String.h>

namespace gazebo {

GazeboResetPlugin::GazeboResetPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboResetPlugin::~GazeboResetPlugin()
{
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboResetPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle;
  publisher_ = node_handle_->advertise<std_msgs::String>("/syscommand", 1);
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboResetPlugin::Reset()
{
  std_msgs::String command;
  command.data = "reset";
  publisher_.publish(command);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboResetPlugin)

} // namespace gazebo
