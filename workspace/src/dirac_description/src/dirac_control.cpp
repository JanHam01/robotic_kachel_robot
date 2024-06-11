#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>

#include "util.h"
#include "a_star/a_star.h"

enum controlStage {
  TURNING,
  DRIVING,
  IDLE,
};






Vector2 desiredPos{0,0};
controlStage currentControlStage = IDLE;
double lastTime = 0;

// Velocity limits
constexpr double maxAngularVel = 3;
constexpr double maxVel = 0.8;

// PID control gains
constexpr double kp = 0.2;
constexpr double ki = 0.15;
constexpr double kd = 0.05;

// Variables for PID control
double errorIntegral = 0.0;
double prevError = 0.0;

// ROS publisher for cmd_vel topic
ros::Publisher cmd_vel_pub;

// Current heading variable
Vector2 currentPos{0.499997,0.500002}; //TODO Current pos: ( 0.499997 , 0.500002 ) 
double currentHeading = 0.0;

double stateChangeTimer = 0;


// Callback function for the PID control
void pidControlCallback(const ros::TimerEvent& event)
{
  // Calculate the desired heading for driving along the circle
  double time = event.current_real.toSec();
  double td = time - lastTime;

  if (lastTime == 0)
  {
    lastTime = time;
    return;
  }
  lastTime = time;

  Vector2 currentToDesired = desiredPos.subtractNew(currentPos);
  double desiredHeading = currentToDesired.getAngle();
  double headingDiff = normalizeAngle(desiredHeading - currentHeading);

  switch(currentControlStage)
  {
      case TURNING:
      {
        double angular_vel = std::max(-maxAngularVel, std::min(headingDiff * 2, maxAngularVel));
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = angular_vel;
        cmd_vel_pub.publish(cmd_vel_msg);

        std::cout << "TURNING - d: " << desiredHeading << " c: " << currentHeading << " diff: " << headingDiff << "\n";
        if (std::abs(desiredHeading - currentHeading) < 0.05)
        {
          currentControlStage = IDLE;
        }
      }
      break;

      case DRIVING:
      {
        double angular_vel = std::max(-maxAngularVel, std::min(headingDiff * 1.5, maxAngularVel));
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = angular_vel;

        double proportionalTerm = kp * currentToDesired.length();
        errorIntegral += currentToDesired.length() * td;
        double integralTerm = ki * errorIntegral;
        double derivativeTerm = kd * (currentToDesired.length() - prevError) / td;
        double pidOutput = std::min(maxVel, proportionalTerm + integralTerm + derivativeTerm);

        prevError = currentToDesired.length();

        cmd_vel_msg.linear.x = pidOutput;

        std::cout << "Current pos: ( " << currentPos.getX() << " , " << currentPos.getY() << " )\n"; 
        std::cout << "Desired pos: ( " << desiredPos.getX() << " , " << desiredPos.getY() << " )\n";
        std::cout << "DRIVING - d: " << desiredHeading << " c: " << currentHeading << " diff: " << headingDiff << "\n";
        std::cout << "DRIVING - d: " << prevError << " PID out: " << pidOutput << "\n";
        if (headingDiff > 0.25) 
        {
          errorIntegral = 0.0;
          prevError = 0.0;
          currentControlStage = TURNING;
        } 
        
        cmd_vel_pub.publish(cmd_vel_msg);

        if (desiredPos.subtractNew(currentPos).length() < 0.1)
        {
           currentControlStage = IDLE;
        }
      }
      break;

      case IDLE:
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel_msg);

      break;
  }
}

// Callback function for the odometry message
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  currentPos = Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y);
  currentHeading = tf::getYaw(msg->pose.pose.orientation);
}

enum dirac_orientation {
  FORWARD = 0,
  RIGHT = 1,
  BACKWARD = 2,
  LEFT = 3,
};

dirac_orientation robot_orientation = FORWARD;

Vector2 orintation_vector[ ] = {Vector2(1.0, 0.0),
                                Vector2(0.0, 1.0),
                                Vector2(-1.0, 0),
                                Vector2(0.0, -1.0)};

constexpr float turning_val = 9999999.0;
Vector2 turn_vector[ ] = {Vector2(turning_val, 0.0),
                                  Vector2(0.0, turning_val),
                                  Vector2(-turning_val, 0),
                                  Vector2(0.0, -turning_val)};

void move_forward() {
  while(currentControlStage != IDLE)ros::spinOnce(); 
  desiredPos = currentPos.addNew(orintation_vector[robot_orientation]);
  currentControlStage = DRIVING;
}

void turn_left() {
  while(currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation + 1) % 4);
  desiredPos = currentPos.addNew(turn_vector[robot_orientation]);

  currentControlStage = TURNING;
}

void turn_right() {
  while(currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation - 1) % 4);
  desiredPos = currentPos.addNew(turn_vector[robot_orientation]);
  
  currentControlStage = TURNING;
}

void turn_arround() {
  while(currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation + 2) % 4);
  desiredPos = currentPos.addNew(turn_vector[robot_orientation]);

  currentControlStage = TURNING;
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "robot_control_node");
  ros::NodeHandle nh;

    robot_orientation = FORWARD;
    currentControlStage = IDLE;

  // Create the ROS publisher for the cmd_vel topic
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/dirac_description/cmd_vel", 10);

  // Create the ROS timer for the PID control loop
  ros::Timer pid_control_timer = nh.createTimer(ros::Duration(1.0 / 60.0), pidControlCallback);

  // Create the ROS subscriber for the odometry topic
  ros::Subscriber odom_sub = nh.subscribe("/dirac_description/odom", 10, odomCallback);

  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();
  move_forward();
  turn_left();
  move_forward();
  move_forward();
  turn_right();
  move_forward();
  move_forward();
  move_forward();
  move_forward();
  turn_arround();

  // Start the ROS node main loop
  ros::spin();
  return 0;
}
