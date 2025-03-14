#include "main.h"

double prev_distance = 0;

void Robot::moveToTargetPose()
{   
    double distance = sqrt(pow(target_pose.x, 2) + pow(target_pose.y, 2));

    // double speed = fmin(0.0005 * distance + 0.0008 * (distance - prev_distance), 0.4);
    double speed = 0.15;

    move_data.speed = speed;
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
    move_data.speed = 0.03 * line_data.chord_length;
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