

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_trajectory");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;

  publisher.publish(twist);
  ros::Duration(5.0).sleep();

  double speed = 3.0;
  double interval = 3.0;

  twist.linear.z = 2.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = -speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = -speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = -1.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  return 0;
}
