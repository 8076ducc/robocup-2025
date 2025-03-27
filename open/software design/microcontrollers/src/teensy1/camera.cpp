#include "main.h"

double regressGoal(double distance)
{
  double regressed_distance = ((0 * pow(distance, 5)) - (0 * pow(distance, 4)) + (0.0002562291 * pow(distance, 3)) - (0.2172749012 * pow(distance, 2)) + (62.9773907121 * distance) - 5797.8985126998);
  return regressed_distance;
}

// store entire goal pose
void Robot::storeGoalPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
  yellow_goal.current_pose.x = yellow_goal_x*2;
  yellow_goal.current_pose.y = yellow_goal_y*2;
  yellow_goal.current_pose.bearing = correctBearing(degrees(atan2(yellow_goal_x, yellow_goal_y)) + robot.current_pose.bearing);

  blue_goal.current_pose.x = blue_goal_x*2;
  blue_goal.current_pose.y = blue_goal_y*2;
  blue_goal.current_pose.bearing = correctBearing(degrees(atan2(blue_goal_x, blue_goal_y)) + robot.current_pose.bearing);
}

void Robot::storeYellowPose(double yellow_goal_x, double yellow_goal_y)
{
  yellow_goal.current_pose.x = yellow_goal_x*2;
  yellow_goal.current_pose.y = yellow_goal_y*2;
  yellow_goal.current_pose.bearing = correctBearing(degrees(atan2(yellow_goal_x, yellow_goal_y)) + robot.current_pose.bearing);
}

void Robot::storeBluePose(double blue_goal_x, double blue_goal_y)
{
  blue_goal.current_pose.x = blue_goal_x*2;
  blue_goal.current_pose.y = blue_goal_y*2;
  blue_goal.current_pose.bearing = correctBearing(degrees(atan2(blue_goal_x, blue_goal_y)) + robot.current_pose.bearing);
}

// store open goal pose
void Robot::storeGoalOpenPose(double yellow_open_x, double yellow_open_y, double blue_open_x, double blue_open_y)
{
  yellow_open.current_pose.x = yellow_open_x;
  yellow_open.current_pose.y = yellow_open_y;
  yellow_open.current_pose.bearing = correctBearing(degrees(atan2(yellow_open_x, yellow_open_y)) + robot.current_pose.bearing);

  blue_open.current_pose.x = blue_open_x;
  blue_open.current_pose.y = blue_open_y;
  blue_open.current_pose.bearing = correctBearing(degrees(atan2(blue_open_x, blue_open_y)) + robot.current_pose.bearing);
}

void Robot::storeYellowOpenPose(double yellow_open_x, double yellow_open_y)
{
  yellow_open.current_pose.x = yellow_open_x;
  yellow_open.current_pose.y = yellow_open_y;
  yellow_open.current_pose.bearing = correctBearing(degrees(atan2(yellow_open_x, yellow_open_y)) + robot.current_pose.bearing);
}

void Robot::storeBlueOpenPose(double blue_open_x, double blue_open_y)
{
  blue_open.current_pose.x = blue_open_x;
  blue_open.current_pose.y = blue_open_y;
  blue_open.current_pose.bearing = correctBearing(degrees(atan2(blue_open_x, blue_open_y)) + robot.current_pose.bearing);
}

void Robot::storeRobotPose()
{
  double yellow_goal_real_dist = regressGoal(sqrt(pow(yellow_goal.current_pose.x, 2) + pow(yellow_goal.current_pose.y, 2)));
  double blue_goal_real_dist = regressGoal(sqrt(pow(blue_goal.current_pose.x, 2) + pow(blue_goal.current_pose.y, 2)));

  robot.current_pose.x = -(blue_goal_real_dist * sin(radians(blue_goal.current_pose.bearing)) + yellow_goal_real_dist * sin(radians(yellow_goal.current_pose.bearing))) / 2;
  robot.current_pose.y = -(blue_goal_real_dist * cos(radians(blue_goal.current_pose.bearing)) + yellow_goal_real_dist * cos(radians(yellow_goal.current_pose.bearing))) / 2;

  // double ema_const = 0.2;
  // robot.current_pose.x = (robot.current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  // robot.current_pose.y = (robot.current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = robot.current_pose.x;
  previous_pose.y = robot.current_pose.y;
}
