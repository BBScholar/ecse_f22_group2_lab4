
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/shared_ptr.hpp>

boost::shared_ptr<ros::Publisher> cmd_vel_pub;

void laser_callback(const sensor_msgs::LaserScan &msg) {}

void des_vel_callback(const geometry_msgs::Twist &msg) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "smart_teleop");

  ros::NodeHandle nh;

  ros::Subscriber des_vel_sub, laser_sub;
  ros::Publisher cmd_vel_pub;

  cmd_vel_pub =
      boost::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::Twist>());

  ros::spin();
  return 0;
}
