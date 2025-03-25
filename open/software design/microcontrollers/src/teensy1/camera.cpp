#include "main.h"

double field_y = 2190;
double field_x = 1580;

double regressGoal(double distance)
{
  regressed_distance = ((0 * pow(distance, 5)) - (0 * pow(distance, 4)) + (0.0002562291 * pow(distance, 3)) - (0.2172749012 * pow(distance, 2)) + (62.9773907121 * distance) - 5797.8985126998);
  return regressed_distance;
}

// store entire goal pose
void Robot::storeGoalPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
  yellow_goal.current_pose.x = regressGoal(yellow_goal_x);
  yellow_goal.current_pose.y = regressGoal(yellow_goal_y);
  yellow_goal.current_pose.bearing = degrees(atan2(yellow_goal_x, yellow_goal_y));

  blue_goal.current_pose.x =  regressGoal(blue_goal_x);
  blue_goal.current_pose.y = regressGoal(blue_goal_y);
  blue_goal.current_pose.bearing = degrees(atan2(blue_goal_x, blue_goal_y));
}

void Robot::storeYellowPose(double yellow_goal_x, double yellow_goal_y)
{
  yellow_goal.current_pose.x = regressGoal(yellow_goal_x);
  yellow_goal.current_pose.y = regressGoal(yellow_goal_y);
  yellow_goal.current_pose.bearing = degrees(atan2(yellow_goal_x, yellow_goal_y));
}

void Robot::storeBluePose(double blue_goal_x, double blue_goal_y)
{
  blue_goal.current_pose.x =  regressGoal(blue_goal_x);
  blue_goal.current_pose.y = regressGoal(blue_goal_y);
  blue_goal.current_pose.bearing = degrees(atan2(blue_goal_x, blue_goal_y));
}

// store open goal pose
void Robot::storeGoalOpenPose(double yellow_open_x, double yellow_open_y, double blue_open_x, double blue_open_y)
{
  yellow_open.current_pose.x = regressGoal(yellow_open_x);
  yellow_open.current_pose.y = regressGoal(yellow_open_y);
  yellow_open.current_pose.bearing = degrees(atan2(yellow_open_x, yellow_open_y));

  blue_open.current_pose.x = regressGoal(blue_open_x);
  blue_open.current_pose.y = regressGoal(blue_open_y);
  blue_open.current_pose.bearing = degrees(atan2(blue_open_x, blue_open_y));
}

void Robot::storeYellowOpenPose(double yellow_open_x, double yellow_open_y)
{
  yellow_open.current_pose.x = regressGoal(yellow_open_x);
  yellow_open.current_pose.y = regressGoal(yellow_open_y);
  yellow_open.current_pose.bearing = degrees(atan2(yellow_open_x, yellow_open_y));
}

void Robot::storeBlueOpenPose(double blue_open_x, double blue_open_y)
{
  blue_open.current_pose.x = regressGoal(blue_open_x);
  blue_open.current_pose.y = regressGoal(blue_open_y);
  blue_open.current_pose.bearing = degrees(atan2(blue_open_x, blue_open_y));
}

void Robot::storeRobotPose()
{
  robot.current_pose.x = -(blue_goal.current_pose.x + yellow_goal.current_pose.x) / 2;
  robot.current_pose.y = -(blue_goal.current_pose.y + yellow_goal.current_pose.y) / 2;

  double ema_const = 0.2; // use 1 for testing purposes
  robot.current_pose.x = (robot.current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  robot.current_pose.y = (robot.current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = robot.current_pose.x;
  previous_pose.y = robot.current_pose.y;
}
