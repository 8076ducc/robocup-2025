#include "main.h"

// double regressBall(double distance)
// { 
//   double regressed_distance = ((0.0000002708 * pow(distance, 5)) - (00.0000696185 * pow(distance, 4)) + (0.0072414285 * pow(distance, 3)) - (0.3554100023 * pow(distance, 2)) + (14.2236266012 * distance) - 104.1068847374);
//   return regressed_distance; 
// }

// double regressGoal(double distance, double goal_y)
// {
//   if (goal_y > 0)
//   {
//     // forwards (positive)
//     double regressed_distance = ((0.0000044237 * pow(distance, 5)) - (0.0020908007 * pow(distance, 4)) + (0.3911084326 * pow(distance, 3)) - (36.0033960558 * pow(distance, 2)) + (1633.3529021812 * distance) - 29028.62896043);
//     return regressed_distance; 
//   }
//   else
//   {
//     // backwards (negative)
//     double regressed_distance = ( - (0.0000014120 * pow(distance, 5)) + (0.0007693469 * pow(distance, 4)) - (0.1555893111 * pow(distance, 3)) + (15.0162772797 * pow(distance, 2)) - (691.0531306509 * distance) + 12345.129926796);
//     return regressed_distance;
//   }
// }

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