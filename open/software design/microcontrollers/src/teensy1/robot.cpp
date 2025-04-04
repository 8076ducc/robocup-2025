#include "main.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

const double STEP_SIZE = 100.0; // Set step size to 10

// Function f(x) = -5 * 10^-15 * x^6 - 700
double f(double x) {
    return -5e-15 * pow(x, 6) - 650;
}

// First derivative of f(x)
double df(double x) {
    return -30e-15 * pow(x, 5);
}

// Second derivative of f(x)
double d2f(double x) {
    return -150e-15 * pow(x, 4);
}

struct Point {
    double x, y;
};

// Compute Euclidean distance between two points
double distance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Find the next point toward the target with a limited step size
Point findNextPoint(Point current, Point target, double step_size) {
    // Ensure current is on the graph
    current.y = f(current.x);
    
    // Compute gradient direction
    double slope = df(current.x);
    double dx = step_size / sqrt(1 + slope * slope);
    dx = (target.x > current.x) ? dx : -dx; // Move toward target
    
    double max_dx = fabs(target.x - current.x); // Limit dx to not exceed error
    if (fabs(dx) > max_dx) {
        dx = (dx > 0) ? max_dx : -max_dx;
    }
    
    double new_x = current.x + dx;
    double new_y = f(new_x);
    
    return {new_x, new_y};
}

// Newton's Method to find the closest x on the curve
double findClosestX(double x0, double y0, double x_init, int max_iter = 50, double tol = 1e-6) {
    double x = x_init;
    for (int i = 0; i < max_iter; i++) {
        double y = f(x);
        double dy_dx = df(x);
        double d2y_dx2 = d2f(x);
        
        double grad = 2 * (x - x0) + 2 * (y - y0) * dy_dx;
        double hessian = 2 + 2 * pow(dy_dx, 2) + 2 * (y - y0) * d2y_dx2;
        
        if (fabs(grad) < tol) break; // Stop if gradient is small
        
        x -= grad / hessian; // Newton update step
    }
    return x;
}

double prev_distance = 0;
double previous_error = 0;
// double previous_

void Robot::moveToPoint(double x, double y, double bearing, double min_speed=0.15, double max_speed = 0.35)
{
    double kp = 0.00045;

    double target_x = x - robot.current_pose.x;
    double target_y = y - robot.current_pose.y;
    // Serial.println("target x: " + String(target_x) + " target y: " + String(target_y));
    double distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

    double speed = bound(kp * distance, min_speed, max_speed);


    move_data.speed = (distance == 0) ? 0 : speed;
    move_data.target_angle = xyToBearing(target_x, target_y);
    // Serial.println("target angle: " + String(move_data.target_angle));
    move_data.target_bearing = bearing;
    // move_data.ema_constant = 0.0002;
    prev_distance = distance;
}


void Robot::goalieTrack()
{   
    // TUNE THIS
    double min_speed = 0.1;
    if (robot.current_pose.y > -500) {
        min_speed = 0.2;
    }

    double max_speed = 0.45;
    // double decel_f = 40;
    // double decel_k = 0.08;

    double Kp = 0.001;
    double Ki = 0; 
    double Kd = 0.00001;  // PID coefficients
    // END TUNE

    double distance = sqrt(pow(ball.current_pose.x, 2) + pow(ball.current_pose.y, 2));
    double target_x, target_y;
    double target_angle, target_bearing;
    
    Point current_pos, target_pos;

    current_pos.x = findClosestX(robot.current_pose.x, robot.current_pose.y, robot.current_pose.x);
    current_pos.y = f(current_pos.x);

    if (ball.detected) {
        target_pos.x = findClosestX(ball.current_pose.x+robot.current_pose.x, ball.current_pose.y+robot.current_pose.y, ball.current_pose.x+robot.current_pose.x);
        target_pos.y = f(target_pose.x);
    } else {
        target_pos.x = 0;
        target_pos.y = f(target_pos.x);
    }
    

    double angle = ball.current_pose.bearing;

    if (angle > 180)
    {
        angle -= 360;
        angle = abs(angle);
    }

    if (angle > 90) {
        angle -= 180;
        angle = abs(angle);
    }

    
    double step_mutli = 1;
    
    if (angle > 35) {
        step_mutli = 3;
    }

    // Serial.print("angle: " + String(angle) + " error: " + String(error) + " speed: " + String(speed));
    
    Point result;
    result=findNextPoint(current_pos, target_pos, step_mutli*STEP_SIZE);

    double error = bound((15*exp(0.2*angle-5))+4*angle + 20/abs(ball.current_pose.y), 0, 300);

    // Proportional term
    double proportional = Kp * error;

    // Derivative term
    double derivative = Kd * (error - previous_error);

    // Store the current error for the next iteration
    previous_error = error;

    // PID output is the sum of the proportional, integral, and derivative terms
    if (result.y < -900) {
        max_speed = 0.2;
    }

    double edge_a = 3.4;
    double edge_b = 0.007;
    double edge_c = 0.0;
    double edge_d = 0.2;

    double average_goal_x;


    if (yellow_goal.detected && blue_goal.detected)
    {
        average_goal_x = abs((yellow_goal.current_pose.x + blue_goal.current_pose.x) / 2);
    }
    else if (yellow_goal.detected)
    {
        average_goal_x = abs(yellow_goal.current_pose.x);
    }
    else if (blue_goal.detected)
    {
        average_goal_x = abs(blue_goal.current_pose.x);
    }
    else
    {
        average_goal_x = 0;
    }

    double max_speed_scaled = fmin(edge_a * exp((-edge_b * average_goal_x) + edge_c) + edge_d, max_speed);
    
    double speed = bound(proportional + derivative, min_speed, max_speed_scaled);

    if (!ball.detected) {
        speed = 0.3;
    }

    robot.moveToPoint(result.x, bound(result.y, -1100, 0), 0, speed, speed);
}

void Robot::goalieRush()
{
    double angle = ball.current_pose.bearing;

    if (angle > 180)
    {
        angle -= 360;
        angle = abs(angle);
    }

    double min_speed;
    if (ball.in_catchment) {
        min_speed = 0.4;
    } else {
        min_speed = 0.2;
    }
    
    robot.moveToPoint(ball.current_pose.x+robot.current_pose.x, ball.current_pose.y+robot.current_pose.y - bound(angle/5*70, 0, 300), 0, min_speed, 0.45);
}

void Robot::trackLine(double speed, double angle, int offset)
{
    double correction = 0;

    offset = angle > 180 ? offset : -offset;

    double ldr_start_angle = line_data.ldr_angles[7 + offset][31 - (int)line_data.line_start_ldr] + current_pose.bearing;
    double ldr_end_angle = line_data.ldr_angles[7 + offset][31 - (int)line_data.line_end_ldr] + current_pose.bearing;

    double ldr_start_correct_angle = correctBearing(angle - ldr_start_angle);
    double ldr_end_correct_angle = correctBearing(angle - ldr_end_angle);

    if (ldr_start_correct_angle > 180)
    {
        ldr_start_correct_angle -= 360;
    }
    else if (ldr_start_correct_angle < -180)
    {
        ldr_start_correct_angle += 360;
    }

    if (ldr_end_correct_angle > 180)
    {
        ldr_end_correct_angle -= 360;
    }
    else if (ldr_end_correct_angle < -180)
    {
        ldr_end_correct_angle += 360;
    }

    if (abs(ldr_start_correct_angle) < abs(ldr_end_correct_angle))
    {
        correction = ldr_start_angle;
    }
    else
    {
        correction = ldr_end_angle;
    }

    move_data.speed = speed;
    move_data.target_angle = correctBearing(correction);
    move_data.target_bearing = 0;
    move_data.ema_constant = 0.0002;
}

void Robot::rejectLine(double bearing)
{
    move_data.speed = 0.06 * line_data.chord_length;
    move_data.target_angle = correctBearing(line_data.line_angle + 180);
    move_data.target_bearing = bearing;
    move_data.ema_constant = 0.005;
}

void Robot::trackLineGoalie(double speed, double angle, int offset)
{
    double correction = 0;

    offset = angle > 180 ? offset : -offset;

    double ldr_start_angle = line_data.ldr_angles[7 + offset][31 - (int)line_data.line_start_ldr] + current_pose.bearing;
    double ldr_end_angle = line_data.ldr_angles[7 + offset][31 - (int)line_data.line_end_ldr] + current_pose.bearing;

    double ldr_start_correct_angle = correctBearing(angle - ldr_start_angle);
    double ldr_end_correct_angle = correctBearing(angle - ldr_end_angle);

    if (ldr_start_correct_angle > 180)
    {
        ldr_start_correct_angle -= 360;
    }
    else if (ldr_start_correct_angle < -180)
    {
        ldr_start_correct_angle += 360;
    }

    if (ldr_end_correct_angle > 180)
    {
        ldr_end_correct_angle -= 360;
    }
    else if (ldr_end_correct_angle < -180)
    {
        ldr_end_correct_angle += 360;
    }

    if (abs(ldr_start_correct_angle) < abs(ldr_end_correct_angle))
    {
        correction = correctBearing(ldr_start_angle);
    }
    else
    {
        correction = correctBearing(ldr_end_angle);
    }

    // if (correctBearing(correction) < 165 || correctBearing(correction) > 195)
    // {
    //     move_data.speed = speed;
    //     move_data.target_angle = correctBearing(correction);
    //     move_data.target_bearing = 0;
    // }
    // else
    // {
    //     move_data.speed = 0;
    //     move_data.target_angle = 0;
    //     move_data.target_bearing = 0;
    // }

    if (abs(correction - 180) < 20)
    {
        move_data.speed = 0;
        move_data.target_angle = correction - 180 > 0 ? 130 : 230;
        move_data.target_bearing = 0;
        move_data.ema_constant = 0.0002;
    }
    else
    {
        move_data.speed = speed;
        move_data.target_angle = correctBearing(correction);
        move_data.target_bearing = 0;
        move_data.ema_constant = 0.0002;
    }
}