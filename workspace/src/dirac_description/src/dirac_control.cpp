#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Int32.h>

#include "util.h"
#include "a_star/a_star.h"

enum controlStage {
  TURNING,
  DRIVING,
  IDLE,
};

Vector2 desiredPos{0, 0};
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
Vector2 currentPos{0.499997, 0.500002}; //TODO Current pos: ( 0.499997 , 0.500002 )
//Vector2 origin_offset{0.499997,0.500002};
//Vector2 idealisedPos{0.0, 0.0};
double currentHeading = 0.0;

double stateChangeTimer = 0;


int front_distance = 0;
int left_distance = 0;
int right_distance = 0;

// Callback function for the PID control
void pidControlCallback(const ros::TimerEvent &event) {
  // Calculate the desired heading for driving along the circle
  double time = event.current_real.toSec();
  double td = time - lastTime;

  if (lastTime == 0) {
    lastTime = time;
    return;
  }
  lastTime = time;

  Vector2 currentToDesired = desiredPos - currentPos;
  double desiredHeading = currentToDesired.getAngle();
  double headingDiff = normalizeAngle(desiredHeading - currentHeading);

  switch (currentControlStage) {
    case TURNING: {
      double angular_vel = std::max(-maxAngularVel, std::min(headingDiff * 2, maxAngularVel));
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = 0;
      cmd_vel_msg.angular.z = angular_vel;
      cmd_vel_pub.publish(cmd_vel_msg);

      //std::cout << "TURNING - d: " << desiredHeading << " c: " << currentHeading << " diff: " << headingDiff << "\n";
      if (std::abs(desiredHeading - currentHeading) < 0.05) {
        std::cout << "END OF TURNING\n";
        std::cout << "Current pos: ( " << currentPos.getX() << " , " << currentPos.getY() << " )\n";
        std::cout << "Desired pos: ( " << desiredPos.getX() << " , " << desiredPos.getY() << " )\n\n";
        currentControlStage = IDLE;
      }
    }
    break;

    case DRIVING: {
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

      //std::cout << "Current pos: ( " << currentPos.getX() << " , " << currentPos.getY() << " )\n";
      //std::cout << "Desired pos: ( " << desiredPos.getX() << " , " << desiredPos.getY() << " )\n";
      //std::cout << "DRIVING - d: " << desiredHeading << " c: " << currentHeading << " diff: " << headingDiff << "\n";
      //std::cout << "DRIVING - d: " << prevError << " PID out: " << pidOutput << "\n";
      /*if (headingDiff > 0.5)
      {
        errorIntegral = 0.0;
        prevError = 0.0;
        currentControlStage = TURNING;
      } */

      cmd_vel_pub.publish(cmd_vel_msg);

      if ((desiredPos - currentPos).length() < 0.3) {
        std::cout << "END OF DRIVING\n";
        std::cout << "Current pos: ( " << currentPos.getX() << " , " << currentPos.getY() << " )\n";
        std::cout << "Desired pos: ( " << desiredPos.getX() << " , " << desiredPos.getY() << " )\n\n";
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
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  currentPos = Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y);
  currentHeading = tf::getYaw(msg->pose.pose.orientation);
}

void frontDistanceCallback(const std_msgs::Int32::ConstPtr &msg) {
  front_distance = msg->data;
}

void leftDistanceCallback(const std_msgs::Int32::ConstPtr &msg) {
  left_distance = msg->data;
}

void rightDistanceCallback(const std_msgs::Int32::ConstPtr &msg) {
  right_distance = msg->data;
}

enum dirac_orientation {
  FORWARD = 0,
  RIGHT = 1,
  BACKWARD = 2,
  LEFT = 3,
};

dirac_orientation robot_orientation = FORWARD;

Vector2 orintation_vector[] = {
  Vector2(1.0, 0.0),
  Vector2(0.0, 1.0),
  Vector2(-1.0, 0),
  Vector2(0.0, -1.0)
};

constexpr float turning_val = 9999999.0;
Vector2 turn_vector[] = {
  Vector2(turning_val, 0.0),
  Vector2(0.0, turning_val),
  Vector2(-turning_val, 0),
  Vector2(0.0, -turning_val)
};

void move_forward() {
  while (currentControlStage != IDLE)ros::spinOnce();
  //idealisedPos = idealisedPos.addNew(orintation_vector[robot_orientation]);
  //desiredPos = idealisedPos.addNew(origin_offset);
  desiredPos = currentPos + orintation_vector[robot_orientation];
  desiredPos = Vector2(((static_cast<int>(desiredPos.getX()) / 1) + 0.5), // NOLINT(*-integer-division)
                       (static_cast<int>(desiredPos.getY()) / 1) + 0.5); // NOLINT(*-integer-division)
  currentControlStage = DRIVING;
}

void turn_left() {
  while (currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation + 1) % 4);
  desiredPos = currentPos + turn_vector[robot_orientation];

  currentControlStage = TURNING;
}

void turn_right() {
  while (currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation - 1) % 4);
  desiredPos = currentPos + turn_vector[robot_orientation];

  currentControlStage = TURNING;
}

void turn_arround() {
  while (currentControlStage != IDLE)ros::spinOnce();
  desiredPos = currentPos;
  robot_orientation = static_cast<dirac_orientation>((robot_orientation + 2) % 4);
  desiredPos = currentPos + turn_vector[robot_orientation];

  currentControlStage = TURNING;
}

int getCurrentX() {
  return static_cast<int>(std::round(currentPos.getX()));
}

int getCurrentY() {
  return static_cast<int>(std::round(currentPos.getY()));
}

Vector2 getObsPlace(int mode) {
  double x;
  double y;
  if (mode == 1) {
    x = getCurrentX() + front_distance * (orintation_vector[robot_orientation].getX());
    y = getCurrentY() + front_distance * (orintation_vector[robot_orientation].getY());
  }
  if (mode == 2) {
    x = getCurrentX() + left_distance * (orintation_vector[(robot_orientation + 1) % 4].getX());
    y = getCurrentY() + left_distance * (orintation_vector[(robot_orientation + 1) % 4].getY());
  }
  if (mode == 3) {
    x = getCurrentX() + right_distance * (orintation_vector[(robot_orientation + 3) % 4].getX());
    y = getCurrentY() + right_distance * (orintation_vector[(robot_orientation + 3) % 4].getY());
  }
  return Vector2(x, y);
}

void fillObstacles(std::vector<Vector2> &currentObs) {
  if (front_distance > 0) {
    auto obsPos = getObsPlace(1);
    if (std::find(currentObs.begin(), currentObs.end(), obsPos) == currentObs.end()) {
      currentObs.push_back(obsPos);
    }
  }
  if (left_distance > 0) {
    auto obsPos = getObsPlace(2);
    if (std::find(currentObs.begin(), currentObs.end(), obsPos) == currentObs.end()) {
      currentObs.push_back(obsPos);
    }
  }
  if (right_distance > 0) {
    auto obsPos = getObsPlace(3);
    if (std::find(currentObs.begin(), currentObs.end(), obsPos) == currentObs.end()) {
      currentObs.push_back(obsPos);
    }
  }
}

dirac_orientation whichWayToTurn(Vector2 dir) {
  dirac_orientation dirToDIrac;
  if (dir.getX() > 0 && dir.getY() == 0) {
    dirToDIrac = FORWARD;
  } else if (dir.getX() < 0 && dir.getY() == 0) {
    dirToDIrac = BACKWARD;
  } else if (dir.getX() == 0 && dir.getY() < 0) {
    dirToDIrac = LEFT;
  } else if (dir.getX() == 0 && dir.getY() > 0) {
    dirToDIrac = RIGHT;
  }
  return static_cast<dirac_orientation>((dirToDIrac - robot_orientation) % 4);
}

AStar theAStar;
Vector2 endPos(5.0, 5.0);
std::vector<Vector2> obstacles;
std::vector<Vector2> path;
bool ready = false;

void controlCallback(const ros::TimerEvent &) {
  if (!ready) {
    return;
  }
  if (getCurrentX() == static_cast<int>(endPos.getX()) && getCurrentY() == static_cast<int>(endPos.getY())) {
    ROS_INFO("Goal reached!");
    return;
  }
  fillObstacles(obstacles);

  for (auto car: obstacles) {
    std::cout << "Obstacle " << car.getX() << " " << car.getY() << std::endl;
  }

  std::cout << "Current pos: ( " << currentPos.getX() << " , " << currentPos.getY() << " )\n";

  std::cout << "End pos: ( " << endPos.getX() << " , " << endPos.getY() << " )\n";

  path.clear();
  path = theAStar.getPath(currentPos, endPos, obstacles);

  std::cout << currentPos.getX() << std::endl;
  for (auto &p: path)
    std::cout << "Path " << p.getX() << " " << p.getY() << std::endl;


  auto dir = path.front();
  dirac_orientation turn = whichWayToTurn(dir);
  std::cout << "make " << turn << std::endl;

  if (turn == LEFT) {
    turn_left();
  } else if (turn == RIGHT) {
    turn_right();
  } else if (turn == BACKWARD) {
    turn_arround();
  }
  for (int i = 0; i < dir.getX() + dir.getY(); i++) {
    move_forward();
  }
  ros::spinOnce();
}


int main(int argc, char **argv) {
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

  ros::Subscriber front_dist_sub = nh.subscribe("/front_distance", 10, frontDistanceCallback);
  ros::Subscriber left_dist_sub = nh.subscribe("/left_distance", 10, leftDistanceCallback);
  ros::Subscriber right_dist_sub = nh.subscribe("/right_distance", 10, rightDistanceCallback);

  ROS_INFO("Waiting for the first odometry message...");
  nav_msgs::OdometryConstPtr init_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/dirac_description/odom");
  if (init_msg != nullptr) {
    ROS_INFO("Received first odometry message, starting control loop.");
    ready = true;
  }

  ros::Timer control_timer = nh.createTimer(ros::Duration(10.0), controlCallback);

  // Start the ROS node main loop
  ros::spin();
  return 0;
}
