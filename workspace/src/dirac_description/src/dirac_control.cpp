#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>

// Constants for the circle
const double circle_radius = 0.4;
const double circle_speed = 0.25;
const double loop_rate_hz = 10.0;

// Velocity limits
const double max_angular_vel = 3;
const double max_vel = 0.7;

// PID control gains
const double kp = 0.5;
const double ki = 0.1;
const double kd = 0.2;

// Variables for PID control
double error_integral_heading = 0.0;
double prev_error_heading = 0.0;

// ROS publisher for cmd_vel topic
ros::Publisher cmd_vel_pub;

// Current heading variable
double current_heading = 0.0;

// Callback function for the PID control
void pidControlCallback(const ros::TimerEvent& event)
{
  // Calculate the desired heading for driving along the circle
  double time = event.current_real.toSec();
  double desired_heading = circle_speed * time;

  // Normalize the desired heading to be between -pi and pi
  while (desired_heading > M_PI)
    desired_heading -= 2 * M_PI;
  while (desired_heading < -M_PI)
    desired_heading += 2 * M_PI;

  // Calculate the heading error
  double heading_error = desired_heading - current_heading;

  // Normalize the heading error to be between -pi and pi
  while (heading_error > M_PI)
    heading_error -= 2 * M_PI;
  while (heading_error < -M_PI)
    heading_error += 2 * M_PI;

  // Calculate the proportional, integral, and derivative terms for the heading
  double proportional_term_heading = kp * heading_error;
  error_integral_heading += heading_error / loop_rate_hz;
  double integral_term_heading = ki * error_integral_heading;
  double derivative_term_heading = kd * (heading_error - prev_error_heading) * loop_rate_hz;

  // Calculate the PID output for the heading
  double pid_output_heading = proportional_term_heading + integral_term_heading + derivative_term_heading;

  // Limit the angular velocity to a maximum value
  double angular_vel = pid_output_heading;
  if (angular_vel > max_angular_vel)
    angular_vel = max_angular_vel;
  else if (angular_vel < -max_angular_vel)
    angular_vel = -max_angular_vel;

  // Create and publish the Twist message with the calculated velocities
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = max_vel;
  cmd_vel_msg.angular.z = angular_vel;
  cmd_vel_pub.publish(cmd_vel_msg);

  // Update the previous error and heading
  prev_error_heading = heading_error;

  std::cout << " " << desired_heading << " "<< current_heading << " " << angular_vel<<  std::endl;
}

// Callback function for the odometry message
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //msg->pose.pose.position.x;
  //msg->pose.pose.position.y;
  // Update the current heading from the odometry message
  current_heading = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "dirac_control_node");
  ros::NodeHandle nh;

  // Create the ROS publisher for the cmd_vel topic
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/dirac_description/cmd_vel", 10);

  // Create the ROS timer for the PID control loop
  ros::Timer pid_control_timer = nh.createTimer(ros::Duration(1.0 / loop_rate_hz), pidControlCallback);

  // Create the ROS subscriber for the odometry topic
  ros::Subscriber odom_sub = nh.subscribe("/dirac_description/odom", 10, odomCallback);

  // Start the ROS node main loop
  ros::spin();

  return 0;
}
