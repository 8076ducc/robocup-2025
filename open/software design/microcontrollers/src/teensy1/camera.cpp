#include "main.h"

void Robot::storeCameraPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
  yellow_goal.current_pose.x = yellow_goal_x;
  yellow_goal.current_pose.y = yellow_goal_y;
  yellow_goal.current_pose.bearing = degrees(atan2(yellow_goal_x, yellow_goal_y));

  blue_goal.current_pose.x = blue_goal_x;
  blue_goal.current_pose.y = blue_goal_y;
  blue_goal.current_pose.bearing = degrees(atan2(blue_goal_x, blue_goal_y));

  // Serial.println(blue_goal.current_pose.y);
  // Serial.println(blue_goal.current_pose.y);
  // Serial.println(blue_goal.current_pose.y - ball.current_pose.y);
  // Serial.println(blue_goal.current_pose.x - ball.current_pose.x);
}

void Robot::storeYellowPose(double yellow_goal_x, double yellow_goal_y)
{
  yellow_goal.current_pose.x = yellow_goal_x;
  yellow_goal.current_pose.y = yellow_goal_y;
  yellow_goal.current_pose.bearing = degrees(atan2(yellow_goal_x, yellow_goal_y));
}

void Robot::storeBluePose(double blue_goal_x, double blue_goal_y)
{
  blue_goal.current_pose.x = blue_goal_x;
  blue_goal.current_pose.y = blue_goal_y;
  blue_goal.current_pose.bearing = degrees(atan2(blue_goal_x, blue_goal_y));
}

void Robot::getRobotPose()
{

  robot.current_pose.x = camera_pose.x;
  robot.current_pose.y = camera_pose.y;

  // tune this to get smooth

  double ema_const = 0.2; // use 1 for testing purposes

  current_pose.x = (current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  current_pose.y = (current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
}