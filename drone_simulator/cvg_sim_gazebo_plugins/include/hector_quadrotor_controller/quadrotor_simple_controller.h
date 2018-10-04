

#ifndef HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
#define HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ardrone_autonomy/Navdata.h>

#define UNKNOWN_MODEL       0
#define INITIALIZE_MODEL    1
#define LANDED_MODEL        2
#define FLYING_MODEL        3
#define HOVERING_MODEL      4
#define TESTING_MODEL       5
#define TAKINGOFF_MODEL     6
#define TO_FIX_POINT_MODEL  7
#define LANDING_MODEL       8
#define LOOPING_MODEL       9

namespace gazebo
{

class GazeboQuadrotorSimpleController : public ModelPlugin
{
public:
  GazeboQuadrotorSimpleController();
  virtual ~GazeboQuadrotorSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;

  // extra robot navi info subscriber
  std::string navdata_topic_;
  ros::Subscriber navdata_subscriber_;
  unsigned int navi_state;
  //***********************************
  
  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  // callback functions for subscribers
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);
  void NavdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::Time state_stamp;
  math::Pose pose;
  math::Vector3 euler, velocity, acceleration, angular_velocity;

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string imu_topic_;
  std::string state_topic_;
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  class PIDController {
  public:
    PIDController();
    virtual ~PIDController();
    virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

    double gain_p;
    double gain_i;
    double gain_d;
    double time_constant;
    double limit;

    double input;
    double dinput;
    double output;
    double p, i, d;

    double update(double input, double x, double dx, double dt);
    void reset();
  };

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
  } controllers_;

  math::Vector3 inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
