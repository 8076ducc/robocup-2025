#include "main.h"
#include <line.h>
#include <RPLidar.h>

RPLidar lidar;

Line front_wall, left_wall, back_wall, right_wall;

// commented out cos its for lidar
// const double y_bounds[2] = {-1215, 1215};
// const double x_bounds[2] = {-910, 910};

double regressedDistance(double distance)
{ 
  double regressed_distance = ((0.0000002708 * pow(distance, 5)) - (00.0000696185 * pow(distance, 4)) + (0.0072414285 * pow(distance, 3)) - (0.3554100023 * pow(distance, 2)) + (14.2236266012 * distance) - 104.1068847374);
  return regressed_distance;  //todo: re-regress in mm
}

// find the pixeldistance from robot to goal (done)
// convert this to actual (done)
// calculate the angle from robot to goal (done)
// find the vector from robot to goal (using distance and bearing)
// do vector sum

void Robot::storeCameraPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
  // ##this shit works##
  double yellow_pixel_distance = sqrt(pow(yellow_goal_x, 2) + pow(yellow_goal_y, 2));
  // Serial.println(yellow_pixel_distance);
  double yellow_actual_distance = regressedDistance(yellow_pixel_distance);
  // Serial.println(yellow_actual_distance);
  double yellow_angle_from_robot = degrees(atan2(regressedDistance(yellow_goal_x), regressedDistance(yellow_goal_y)));
  // Serial.println(yellow_angle_from_robot);

  yellow_goal.current_pose.bearing = correctBearing(yellow_angle_from_robot + robot.current_pose.bearing);
  // Serial.print("yellow goal current pos bearing" + yellow_goal.current_pose.bearing);
  yellow_goal.current_pose.x = sin(radians(yellow_goal.current_pose.bearing)) * yellow_actual_distance;
  yellow_goal.current_pose.y = cos(radians(yellow_goal.current_pose.bearing)) * yellow_actual_distance;

  double blue_pixel_distance = sqrt(pow(blue_goal_x, 2) + pow(blue_goal_y, 2));
  double blue_actual_distance = regressedDistance(blue_pixel_distance);
  double blue_angle_from_robot = degrees(atan2(regressedDistance(blue_goal_x), regressedDistance(blue_goal_y)));
  
  blue_goal.current_pose.bearing = correctBearing(blue_angle_from_robot + robot.current_pose.bearing);
  blue_goal.current_pose.x = sin(radians(blue_goal.current_pose.bearing)) * blue_actual_distance;
  blue_goal.current_pose.y = cos(radians(blue_goal.current_pose.bearing)) * blue_actual_distance;

  Pose centre_of_field;

  //vector to centre of field
  // centre_of_field.x = (blue_goal_x + yellow_goal_x) / 2;
  centre_of_field.x = (blue_goal.current_pose.x + yellow_goal.current_pose.x) / 2;
  // centre_of_field.y = ((abs(yellow_goal_y) + abs(blue_goal_y)) / 2298) * (yellow_goal_y + blue_goal_y) / 2;
  centre_of_field.y = ((abs(yellow_goal.current_pose.y) + abs(blue_goal.current_pose.y)) / 2298) * (yellow_goal.current_pose.y + blue_goal.current_pose.y) / 2;

  // centre_of_field.bearing = degrees(atan2(regressedDistance(centre_of_field.x), regressedDistance(centre_of_field.y)));
  centre_of_field.bearing = correctBearing(degrees(atan2(centre_of_field.x, centre_of_field.y)) + robot.current_pose.bearing);

  double distance_from_centre = sqrt(pow(centre_of_field.x, 2) + pow(centre_of_field.y, 2));

  centre_of_field.x = sin(radians(centre_of_field.bearing)) * distance_from_centre;
  centre_of_field.y = cos(radians(centre_of_field.bearing)) * distance_from_centre;

  camera_pose.x = 1820 / 2 - centre_of_field.x;
  camera_pose.y = 2430 / 2 - centre_of_field.y;

  Serial.print("Camera: x: ");
  Serial.print(camera_pose.x);
  Serial.print(",y: ");
  Serial.println(camera_pose.y);
}

void Robot::getSingleCameraPose(int x, int y)
{
  double angle_from_robot = degrees(atan2(x, y));

  camera_pose.bearing = correctBearing(angle_from_robot + current_pose.bearing);
  camera_pose.x = 1820 / 2 - x;
  camera_pose.y = cos(radians(camera_pose.bearing)) > 0 ? 2298 - y : 0 - y;

  // Serial.print("Camera: ");
  // Serial.print(camera_pose.x);
  // Serial.print(", ");
  // Serial.println(camera_pose.y);
}

void Robot::getRobotPose()
{
  current_pose.x = camera_pose.x;
  current_pose.y = camera_pose.y;

  // print current position
  Serial.print("Robot: ");
  Serial.print(current_pose.x);
  Serial.print(", ");
  Serial.println(current_pose.y);

  //tune this to get smooth
  double ema_const = 0.2; // use 1 for testing purposes


  current_pose.x = (current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  current_pose.y = (current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
}