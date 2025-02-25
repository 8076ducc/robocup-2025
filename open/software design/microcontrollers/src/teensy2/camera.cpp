#include "main.h"
#include <line.h>
#include <RPLidar.h>

RPLidar lidar;

Line front_wall, left_wall, back_wall, right_wall;

// commented out cos its for lidar
// const double y_bounds[2] = {-1215, 1215};
// const double x_bounds[2] = {-910, 910};

double regressBall(double distance)
{ 
  double regressed_distance = ((0.0000002708 * pow(distance, 5)) - (00.0000696185 * pow(distance, 4)) + (0.0072414285 * pow(distance, 3)) - (0.3554100023 * pow(distance, 2)) + (14.2236266012 * distance) - 104.1068847374);
  return regressed_distance; 
}

double regressGoal(double distance)
{ 
  double regressed_distance = ((0.0000038850 * pow(distance, 5)) - (0.0018075922 * pow(distance, 4)) + (0.3300455275 * pow(distance, 3)) - (29.4064192107 * pow(distance, 2)) + (1280.0320598455 * distance) - 21657.4407165921);
  return regressed_distance; 
}


// find the pixeldistance from robot to goal (done)
// convert this to actual (done)
// calculate the angle from robot to goal (done)
// find the vector from robot to goal (using distance and bearing)
// do vector sum

// void Robot::storeCameraPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
// {

//   double yellow_pixel_distance = sqrt(pow(yellow_goal_x, 2) + pow( yellow_goal_y, 2));
//   double yellow_actual_distance = regressGoal(yellow_pixel_distance);
//   double yellow_angle_from_robot = degrees(atan2(regressGoal(yellow_goal_x), regressGoal(yellow_goal_y)));

//   yellow_goal.current_pose.bearing = correctBearing(yellow_angle_from_robot + robot.current_pose.bearing);
//   yellow_goal.current_pose.x = sin(radians(yellow_goal.current_pose.bearing)) * yellow_actual_distance;
//   yellow_goal.current_pose.y = cos(radians(yellow_goal.current_pose.bearing)) * yellow_actual_distance;

//   double blue_pixel_distance = sqrt(pow(blue_goal_x, 2) + pow(blue_goal_y, 2));
//   double blue_actual_distance = regressGoal(blue_pixel_distance);
//   double blue_angle_from_robot = degrees(atan2(regressGoal(blue_goal_x), regressGoal(blue_goal_y)));
  
//   blue_goal.current_pose.bearing = correctBearing(blue_angle_from_robot + robot.current_pose.bearing);
//   blue_goal.current_pose.x = sin(radians(blue_goal.current_pose.bearing)) * blue_actual_distance;
//   blue_goal.current_pose.y = cos(radians(blue_goal.current_pose.bearing)) * blue_actual_distance;

//   Serial.print("y_dist: ");
//   Serial.print(yellow_actual_distance);
//   Serial.print(" b_dist: ");
//   Serial.println(blue_actual_distance);

//   Pose centre_of_field;

//   //vector to centre of field
//   centre_of_field.x = (blue_goal.current_pose.x + yellow_goal.current_pose.x) / 2;
//   centre_of_field.y = ((abs(yellow_goal.current_pose.y) + abs(blue_goal.current_pose.y)) / 2190) * (yellow_goal.current_pose.y + blue_goal.current_pose.y) / 2;
//   centre_of_field.bearing = correctBearing(degrees(atan2(centre_of_field.x, centre_of_field.y)) + robot.current_pose.bearing);

//   // Serial.print("Centre of field: x: ");
//   // Serial.print(centre_of_field.x);
//   // Serial.print(", y: ");
//   // Serial.println(centre_of_field.y);

//   camera_pose.x = 1580 / 2 - centre_of_field.x;
//   camera_pose.y = 2190 / 2 - centre_of_field.y;

//   // Serial.print("Camera: x: ");
//   // Serial.print(camera_pose.x);
//   // Serial.print(", y: ");
//   // Serial.println(camera_pose.y);
// }

void Robot::storeCameraPose(double yellow_goal_x, double yellow_goal_y, double blue_goal_x, double blue_goal_y)
{
    // Compute the center of the field in pixel coordinates
    double center_x = (yellow_goal_x + blue_goal_x) / 2.0;
    double center_y = (yellow_goal_y + blue_goal_y) / 2.0;

    Serial.print(blue_goal_y);
    Serial.print(" ");
    Serial.println(yellow_goal_y);

    // Compute the pixel distance from the robot to the field center
    double center_pixel_distance = sqrt(pow(center_x, 2) + pow(center_y, 2));

    // Compute mm per pixel scale using real-world goal distance (2298mm)
    double goal_pixel_distance = sqrt(pow(blue_goal_x - yellow_goal_x, 2) + pow(blue_goal_y - yellow_goal_y, 2));
    double mm_per_pixel = 2120.0 / goal_pixel_distance;

    // Convert pixel distances to real-world distances
    double center_actual_x = center_x * mm_per_pixel;
    double center_actual_y = center_y * mm_per_pixel;

    // Serial.print("x (cm): ");
    // Serial.print(center_actual_x / 10);
    // Serial.print(" | y (cm): ");
    // Serial.println(center_actual_y / 10);

    // // Convert robot's pixel distance to real-world distance
    // double robot_actual_distance = center_pixel_distance * mm_per_pixel;

    // // Compute angle from robot to center
    // double center_angle_from_robot = degrees(atan2(center_x, center_y));

    // // Adjust for robot's current bearing
    // robot.current_pose.bearing = correctBearing(center_angle_from_robot + robot.current_pose.bearing);

    // // Compute robot's actual position using the adjusted angle
    // robot.current_pose.x = sin(radians(robot.current_pose.bearing)) * robot_actual_distance;
    // robot.current_pose.y = cos(radians(robot.current_pose.bearing)) * robot_actual_distance;

    // Debugging output
    // Serial.print("Pixel distance between goals: ");
    // Serial.println(goal_pixel_distance);
    // Serial.print("mm per pixel: ");
    // Serial.println(mm_per_pixel);
    // Serial.print("Robot dist from center: ");
    // Serial.println(robot_actual_distance);
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
  // Serial.print("Robot: ");
  // Serial.print(current_pose.x);
  // Serial.print(", ");
  // Serial.println(current_pose.y);

  //tune this to get smooth
  double ema_const = 0.2; // use 1 for testing purposes


  current_pose.x = (current_pose.x * ema_const) + (previous_pose.x * (1 - ema_const));
  current_pose.y = (current_pose.y * ema_const) + (previous_pose.y * (1 - ema_const));

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
}