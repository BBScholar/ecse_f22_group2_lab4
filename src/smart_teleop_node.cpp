
#include <ros/console.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

const double default_theta_threshold = M_PI / 2;
const double default_dist_threshold = 0.3;

ros::Publisher *cmd_vel_pub_shared;

geometry_msgs::Twist latest_cmd;

double theta_threshold, dist_threshold;

void laser_callback(const sensor_msgs::LaserScan &msg) {
  const double velocity = latest_cmd.linear.x;

  int min_idx = 0;
  double min_dist = msg.ranges[0];

  for (int i = 0; i < 270; ++i) {
    if (msg.ranges[i] < min_dist) {
      min_dist = msg.ranges[i];
      min_idx = i;
    }
  }

  const double min_angle = msg.angle_min + min_idx * msg.angle_increment;

  geometry_msgs::Twist cmd_vel;

  if ((velocity > 0.0 && min_angle > -theta_threshold &&
       min_angle < theta_threshold && min_dist < dist_threshold) ||
      velocity < 0.0) {
    cmd_vel.linear.x = 0.0;
  } else {
    cmd_vel.linear.x = latest_cmd.linear.x;
  }

  cmd_vel.angular = latest_cmd.angular;

  cmd_vel_pub_shared->publish(cmd_vel);
}

void des_vel_callback(const geometry_msgs::Twist &msg) {
  // rebroadcast
  ROS_INFO("Recieved command.");
  latest_cmd = msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "smart_teleop");

  ros::NodeHandle nh;

  ROS_INFO("Starting smart teleoperation node...");

  ros::Subscriber des_vel_sub, laser_sub;
  ros::Publisher cmd_vel_pub;

  des_vel_sub = nh.subscribe("des_vel", 32, des_vel_callback);
  laser_sub = nh.subscribe("laser_1", 32, laser_callback);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 32);
  cmd_vel_pub_shared = &cmd_vel_pub;

  nh.setParam("theta_threshold", default_theta_threshold);
  nh.setParam("dist_threshold", default_dist_threshold);

  ros::Rate r(5);
  while (ros::ok()) {
    nh.param<double>("theta_threshold", theta_threshold,
                     default_theta_threshold);
    nh.param<double>("dist_threshold", dist_threshold, default_dist_threshold);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
