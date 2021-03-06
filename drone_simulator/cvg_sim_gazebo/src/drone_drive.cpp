

#include "drone_gazebo/drone_drive.h"

DroneDrive::DroneDrive()
  : nh_priv_("~")
{
  //Init gazebo ros node
  ROS_INFO("Simulation Node Init");
  ROS_ASSERT(init());
}

DroneDrive::~DroneDrive()
{
  updatecommandVelocity(0.0, 0.0);
  takeoffCommand();
  ros::shutdown();
}

//Init function
bool DroneDrive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // variables to check range
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.9;
  check_side_dist_    = 0.8;

  //initialize drone pose/prev_pose
  drone_pose_ = 0.0;
  prev_drone_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("ardrone/takeoff",1);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &DroneDrive::laserScanMsgCallBack, this);
  state_sub_ = nh_.subscribe("ground_truth/state", 10, &DroneDrive::stateMsgCallBack, this);

  return true;
}

void DroneDrive::stateMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
  
  ROS_INFO("od %f",siny);
  ROS_INFO("od %f",cosy);
  drone_pose_ = atan2(siny, cosy);
}

//Function for get data from laser in 3 angles
void DroneDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //set laser angle for get data
  uint16_t scan_angle[3] = {540, 680, 400};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

//Function for send command velocity, change location 
void DroneDrive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  ROS_INFO("3 %f",linear);
  cmd_vel_pub_.publish(cmd_vel);
}

//Function to takeoff drone
void DroneDrive::takeoffCommand()
{
  std_msgs::Empty msg;

  takeoff_pub_.publish(msg);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool DroneDrive::controlLoop()
{
  static uint8_t state_num = 0;

  ROS_INFO("scan center %f",scan_data_[CENTER]);
  ROS_INFO("scan left %f",scan_data_[LEFT]);
  ROS_INFO("scan right %f",scan_data_[RIGHT]);
  switch(state_num)
  {
    
    case 0://Direction
      ROS_INFO("state d");
      updatecommandVelocity(0.0, 0.0);
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        ROS_INFO("left %f",scan_data_[LEFT]);
        ROS_INFO("right %f",scan_data_[RIGHT]);
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_drone_pose_ = drone_pose_;
          state_num = 2;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_drone_pose_ = drone_pose_;
          state_num = 3;
        }
        else
        {
          state_num = 1;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_drone_pose_ = drone_pose_;
        state_num = 4;
      }
      break;

    case 1://Forward
      ROS_INFO("state f");
      updatecommandVelocity(1.0, 0.0);
      state_num = 0;
      break;

    case 2://Right
      ROS_INFO("state r");
      ROS_INFO("state r1 %f", prev_drone_pose_);
      ROS_INFO("state r2 %f", drone_pose_);
      if (fabs(prev_drone_pose_ - drone_pose_) >= escape_range_)
        state_num = 0;
      else
      { 
        ROS_INFO("state r done");
        //updatecommandVelocity(0.0, 0.0);
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      }
      break;

    case 3://Left
      ROS_INFO("state l");
      ROS_INFO("state l1 %f", prev_drone_pose_);
      ROS_INFO("state l2 %f", drone_pose_);
      if (fabs(prev_drone_pose_ - drone_pose_) >= escape_range_)
        state_num = 0;
      else
      {
        ROS_INFO("state l done");
        //updatecommandVelocity(0.0, 0.0);
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      }
      break;

    case 4://Turn 90
      ROS_INFO("state turn");
      updatecommandVelocity(0.0, ANGULAR_VELOCITY*3);
      state_num = 0;
    break;

    default:
      state_num = 0;
      break;
  }

  return true;
}

//Main 
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drone_drive");
  DroneDrive drone_drive;

  ros::Rate loop_rate(125);
  ros::Rate r(300);

  while (ros::ok())
  {
    
    drone_drive.takeoffCommand();
    ROS_INFO("Taking off...");
    r.sleep();
    ROS_INFO("We fly");

    drone_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
