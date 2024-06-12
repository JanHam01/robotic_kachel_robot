#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

ros::Publisher front_distance_pub;
ros::Publisher left_distance_pub;
ros::Publisher right_distance_pub;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // Calculate index for front, left, and right based on the scan angle
  int front_index = msg->ranges.size() / 2;
  int left_index = msg->ranges.size() - 1;
  int right_index = 0;

  // Extract distances and convert to integers
  int front_distance = static_cast<int>(msg->ranges[front_index]) + 1;
  int left_distance = static_cast<int>(msg->ranges[left_index]) + 1;
  int right_distance = static_cast<int>(msg->ranges[right_index]) + 1;

  // Publish distances
  std_msgs::Int32 front_distance_msg;
  std_msgs::Int32 left_distance_msg;
  std_msgs::Int32 right_distance_msg;

  front_distance_msg.data = front_distance;
  left_distance_msg.data = left_distance;
  right_distance_msg.data = right_distance;

  front_distance_pub.publish(front_distance_msg);
  left_distance_pub.publish(left_distance_msg);
  right_distance_pub.publish(right_distance_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/lidar_scan", 1000, lidarCallback);
  front_distance_pub = nh.advertise<std_msgs::Int32>("/front_distance", 1000);
  left_distance_pub = nh.advertise<std_msgs::Int32>("/left_distance", 1000);
  right_distance_pub = nh.advertise<std_msgs::Int32>("/right_distance", 1000);

  ros::spin();

  return 0;
}
