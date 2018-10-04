

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

// #define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <cvg_sim_gazebo_plugins/SetBias.h>
#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{
   class GazeboRosIMU : public ModelPlugin
   {
   public:
      /// \brief Constructor
      GazeboRosIMU();

      /// \brief Destructor
      virtual ~GazeboRosIMU();

   protected:
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Reset();
      virtual void Update();

   private:
      /// \brief The parent World
      physics::WorldPtr world;

      /// \brief The link referred to by this plugin
      physics::LinkPtr link;

      /// \brief pointer to ros node
      ros::NodeHandle* node_handle_;
      ros::Publisher pub_;

      /// \brief ros message
      sensor_msgs::Imu imuMsg;

      /// \brief store link name
      std::string linkName;

      /// \brief frame id
      std::string frameId;

      /// \brief topic name
      std::string topicName;

      /// \brief Sensor models
      SensorModel3 accelModel;
      SensorModel3 rateModel;
      SensorModel headingModel;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      boost::mutex lock;

      /// \brief save last_time
      common::Time last_time;
      common::Time update_period;

      /// \brief save current body/physics state
      math::Quaternion orientation;
      math::Vector3 velocity;
      math::Vector3 accel;
      math::Vector3 rate;
      math::Vector3 gravity;
      math::Vector3 gravity_body;

      /// \brief Gaussian noise generator
      double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      std::string robotNamespace;

      /// \brief call back when using service
      bool ServiceCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
      ros::ServiceServer srv_;
      std::string serviceName;

      /// \brief Bias service callbacks
      bool SetAccelBiasCallback(cvg_sim_gazebo_plugins::SetBias::Request &req, cvg_sim_gazebo_plugins::SetBias::Response &res);
      bool SetRateBiasCallback(cvg_sim_gazebo_plugins::SetBias::Request &req, cvg_sim_gazebo_plugins::SetBias::Response &res);
      ros::ServiceServer accelBiasService;
      ros::ServiceServer rateBiasService;

#ifdef USE_CBQ
      ros::CallbackQueue callback_queue_;
      void CallbackQueueThread();
      boost::thread callback_queue_thread_;
#endif

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
   };
}

#endif 