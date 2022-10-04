
#include <ros/console.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/shared_ptr.hpp>

ros::Publisher *cmd_vel_pub_shared;

sensor_msgs::LaserScan latest_laser;

void laser_callback(const sensor_msgs::LaserScan &msg) { latest_laser = msg; }

void des_vel_callback(const geometry_msgs::Twist &msg) {
  // rebroadcast
  ROS_INFO("Republishing message");
  cmd_vel_pub_shared->publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "smart_teleop");

  ROS_INFO("Starting smart teleoperation node...");

  ros::NodeHandle nh;

  ros::Subscriber des_vel_sub, laser_sub;
  ros::Publisher cmd_vel_pub;

  des_vel_sub = nh.subscribe("des_vel", 32, des_vel_callback);
  laser_sub = nh.subscribe("laser_1", 32, laser_callback);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 32);
  cmd_vel_pub_shared = &cmd_vel_pub;

  ros::spin();
  return 0;
}
