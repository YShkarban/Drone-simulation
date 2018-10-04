
#include <hector_gazebo_plugins/gazebo_ros_sonar.h>
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/RaySensor.hh"

#include <limits>

namespace gazebo {

GazeboRosSonar::GazeboRosSonar()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  sensor_->SetActive(false);
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->WorldName();
  world = physics::get_world(worldName);

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameId"))
    frame_id_ = "";
  else
    frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
    topic_ = "sonar";
  else
    topic_ = _sdf->GetElement("topicName")->Get<std::string>();

  sensor_model_.Load(_sdf);

  range_.header.frame_id = frame_id_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_.field_of_view = std::min(fabs((sensor_->AngleMax() - sensor_->AngleMin()).Radian()), fabs((sensor_->VerticalAngleMax() - sensor_->VerticalAngleMin()).Radian()));
  range_.max_range = sensor_->RangeMax();
  range_.min_range = sensor_->RangeMin();

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);

  Reset();
  updateConnection = sensor_->LaserShape()->ConnectNewLaserScans(
        boost::bind(&GazeboRosSonar::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void GazeboRosSonar::Reset()
{
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSonar::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
//  if (last_time + updatePeriod > sim_time) return;

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  range_.header.stamp.sec  = (world->GetSimTime()).sec;
  range_.header.stamp.nsec = (world->GetSimTime()).nsec;

  // find ray with minimal range
  range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  int num_ranges = sensor_->LaserShape()->GetSampleCount() * sensor_->LaserShape()->GetVerticalSampleCount();
  for(int i = 0; i < num_ranges; ++i) {
    double ray = sensor_->LaserShape()->GetRange(i);
    if (ray < range_.range) range_.range = ray;
  }

  // add Gaussian noise (and limit to min/max range)
  if (range_.range < range_.max_range) {
    range_.range += sensor_model_.update(dt);
    if (range_.range < range_.min_range) range_.range = range_.min_range;
    if (range_.range > range_.max_range) range_.range = range_.max_range;
  }

  publisher_.publish(range_);

  // save last time stamp
  last_time = sim_time;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

} // namespace gazebo
