#include "main.h"

double prev_distance = 0;
double previous_error = 0;

void Robot::goalieTrack()
{   
    // TUNE THIS
    double min_speed = 0.1;
    double max_speed = 0.35;
    // double decel_f = 40;
    // double decel_k = 0.08;

    double Kp = 0.00005;
    double Ki = 0; 
    double Kd = 0.00001;  // PID coefficients
    // END TUNE

    double distance = sqrt(pow(target_pose.x, 2) + pow(target_pose.y, 2));

    double error = abs(target_pose.x * ball.current_pose.bearing);

    // Proportional term
    double proportional = Kp * error;

    // Integral term
    double integral;
    integral += error;
    double integralTerm = Ki * integral;

    // Derivative term
    double derivative = Kd * (error - previous_error);

    // Store the current error for the next iteration
    previous_error = error;

    // PID output is the sum of the proportional, integral, and derivative terms
    double speed = bound(proportional + integralTerm + derivative, min_speed, max_speed);

    move_data.speed = (distance == 0) ? 0 : speed;
    move_data.target_angle = xyToBearing(target_pose.x, target_pose.y);
    move_data.target_bearing = correctBearing(target_pose.bearing);
    move_data.ema_constant = 0.0002;
    previous_error = error;
}

void Robot::goalieRush()
{
    double distance = sqrt(pow(target_pose.x, 2) + pow(target_pose.y, 2));
    double speed = 0.3;

    move_data.speed = (distance == 0) ? 0 : speed;
    move_data.target_angle = xyToBearing(target_pose.x, target_pose.y);
    move_data.target_bearing = correctBearing(target_pose.bearing);
    move_data.ema_constant = 0.0002;
    prev_distance = distance;
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