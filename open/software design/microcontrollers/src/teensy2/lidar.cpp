#include "main.h"

const double y_bounds[2] = {-1215, 1215};
const double x_bounds[2] = {-910, 910};

double regressedDistance(double distance)
{
  double regressed_distance = pow(0.0009606260 * distance, 3) - pow(0.1161190388 * distance, 2) + 8.8058432711 * distance;
  return regressed_distance;
}

void Robot::getCameraPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
  Pose centre_of_field;

  centre_of_field.x = (yellow_goal_x + blue_goal_x) / 2;
  centre_of_field.y = ((abs(yellow_goal_y) + abs(blue_goal_y)) / 2298) * (yellow_goal_y + blue_goal_y) / 2;
  centre_of_field.bearing = degrees(atan2(regressedDistance(centre_of_field.x), regressedDistance(centre_of_field.y)));

  double distance_from_centre = sqrt(pow(centre_of_field.x, 2) + pow(centre_of_field.y, 2));

  centre_of_field.bearing += current_pose.bearing;
  centre_of_field.x = sin(radians(centre_of_field.bearing)) * distance_from_centre;
  centre_of_field.y = cos(radians(centre_of_field.bearing)) * distance_from_centre;

  camera_pose.x = 1820 / 2 - centre_of_field.x;
  camera_pose.y = 2430 / 2 - centre_of_field.y;

  Serial.print("Camera: ");
  Serial.print(camera_pose.x);
  Serial.print(", ");
  Serial.println(camera_pose.y);
}

void Robot::getSingleCameraPose(int x, int y)
{
  double angle_from_robot = degrees(atan2(x, y));

  camera_pose.bearing = correctBearing(angle_from_robot + current_pose.bearing);
  camera_pose.x = 1820 / 2 - x;
  camera_pose.y = cos(radians(camera_pose.bearing)) > 0 ? 2298 - y : 0 - y;

  Serial.print("Camera: ");
  Serial.print(camera_pose.x);
  Serial.print(", ");
  Serial.println(camera_pose.y);
}

void Robot::getRobotPose()
{

  current_pose.x = camera_pose.x;
  current_pose.y = camera_pose.y;

  // Serial.print(current_pose.x);
  // Serial.print(", ");
  // Serial.println(current_pose.y);

  //tune this to get smooth
  double ema_const = 0.45;

  current_pose.x = (current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  current_pose.y = (current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
}