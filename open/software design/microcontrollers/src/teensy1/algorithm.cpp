#include "main.h"
Pose target_pose_wrt_goal;
Pose target_pose_wrt_ball;

void Robot::defendGoal()
{
    double target_distance_from_goal = 0;
    target_pose.x = ball.current_pose.x;
    target_pose.y = 0; // target_distance_from_goal;
    target_pose.bearing = 0;
    moveToTargetPose();
}

void Robot::rotateToBall()
{
    target_pose = ball.current_pose;
    target_pose.bearing -= robot.current_pose.bearing;
    moveToTargetPose();
}

void Robot::orbitToBall(double bearing)
{
    // Serial.println("running orbitToBall");
    if (ball.detected)
    {
        // Serial.println("ball: " + String(ball.current_pose.bearing) + " robot(imu): " + String(robot.current_pose.bearing ));
        double bearing_from_robot = correctBearing(ball.current_pose.bearing - robot.current_pose.bearing);
        double offset, multiplier;
        double goal_y = blue_goal.current_pose.y;

        if (bearing_from_robot < 180)
        {
            offset = fmin(bearing_from_robot * 1.05, 90);
        }
        else
        {
            offset = fmax((bearing_from_robot - 360) * 1.05, -90);
        }

#ifdef BOT_A
        // double a = 0.085; // affects orbit radius (shift in and out)
        // double b = 1.7; // pivots the curve
        // double c = 150; // typically represents maximum distance from the ball
        // double d = 1; // maximum multiplier

        // TUNE THIS
        double orbit_a = 0.15;
        double orbit_b = 2.1;
        double orbit_c = 150;
        double orbit_d = 1;
        // END TUNE

        double factor = orbit_d - (ball.distance_from_robot) / orbit_c;

        // f = 1 - D/c
        // M = min(1, ae^bf)
       
        multiplier = fmin(orbit_d, orbit_a * exp(orbit_b * factor));
        // Serial.println(String(multiplier));
        // Serial.print("multiplier: ");
        // Serial.println(multiplier);

#else
        double factor = 1.1 - (ball.distance_from_robot) / 2190;

        multiplier = fmin(1.1, 0.01 * exp(factor * 3.5));
        Serial.print("multiplier: ");
        Serial.println(multiplier);

#endif
        // speed calculation
        // double speed = fmin(max(0.25, 0.00001 * pow(ball.distance_from_robot, 2)), 0.3);

        // TUNE THIS
        double orbit_min_speed = 0.1;
        double orbit_max_speed = 0.4;
        double orbit_decel_f = 65; // typically represents the maximum distance from the ball in pixels
        double orbit_decel_k = 0.065; // increase for faster deceleration
        // END TUNE

        // move slower when close to the ball
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

        // TUNE THIS
        double edge_a = 9;
        double edge_b = 0.03;
        double edge_c = -1.81;
        // END TUNE

        // scale the maximum speed based on the distance from the edge
        double orbit_max_speed_scaled = fmin(edge_a * exp((-edge_b * average_goal_x) + edge_c), orbit_max_speed);
        
        // deceleration curve
        // double speed = min(max(0.01 * ball.distance_from_robot, 0.15),  0.5);
        double speed = fmin(fmax(orbit_decel_k * exp(ball.distance_from_robot / orbit_decel_f), orbit_min_speed), orbit_max_speed_scaled);

        // double speed;
        // if (ball.distance_from_robot > 500)
        // {
        //     speed = 0.35;
        // }
        // else
        // {
        //     speed = 0.25;
        // }

        double correction = correctBearing(bearing_from_robot + multiplier * offset);
        // Serial.print("correction: ");
        // Serial.println(correction);

        if (correction < 10 ||correction > 350){
            correction = 0;
        }

        if (line_data.on_line)
        {
            if (abs(correction - line_data.initial_line_angle) < 90 && (line_data.initial_line_angle > 20 && line_data.initial_line_angle < 330))
            {
                // trackLine(speed, correction, 7);
                rejectLine(bearing);
            }
            else
            {
                // Serial.print("correction: ");
                // Serial.println(correction);
                // Serial.print("line angle: ");
                // Serial.println(line_data.initial_line_angle);
                rejectLine(bearing);
            }
        }
        else
        {
            // TUNE THIS
            double goal_y_diff_thresh = 25;
            double goal_x_diff_thresh = 40;
            // END TUNE

            if (abs(goal_y - ball.current_pose.y) < goal_y_diff_thresh && abs(ball.current_pose.x - average_goal_x) < goal_x_diff_thresh)
            {
                move_data.speed = 0;
            }
            else
            {
                move_data.speed = speed;
            }
            move_data.target_angle = correction;
            move_data.target_bearing = bearing;
            move_data.ema_constant = 0.0002;
        }
    }
    else
    {
        // target_pose.x = 910;
        // target_pose.y = 1215;
        // moveToTargetPose();
        move_data.speed = 0;
        move_data.target_angle = 0;
        move_data.target_bearing = 0;
        move_data.ema_constant = 0.0002;
    }
}

void Robot::orbitScore()
{
    // Serial.println("running orbitScore");
    double target_bearing = robot.dip_4_on ? yellow_goal.current_pose.bearing : blue_goal.current_pose.bearing;
    target_bearing = blue_goal.current_pose.bearing;

    double goal_y = blue_goal.current_pose.y;

    // TUNE THIS
    double score_min_speed = 0.1;
    double score_max_speed = 0.3;
    double score_decel_f = 62;
    double score_decel_k = 0.05;
    // END TUNE

    if (line_data.on_line)
    {
        rejectLine(target_bearing);
        // layer_1_rx_data.data.kick = true;
    }
    else
    {
        move_data.speed = fmin(fmax(score_decel_k * exp(goal_y / score_decel_f), score_min_speed), score_max_speed);
        // if (target_bearing < 0)
        // {
        //     move_data.target_bearing = target_bearing - score_bearing_offset;
        // }
        // else
        // {
        //     move_data.target_bearing = target_bearing + score_bearing_offset;
        // }
        move_data.target_bearing = target_bearing;
        move_data.target_angle = 0;
        move_data.ema_constant = 0.00017;
    }
}

void Robot::rotateScore()
{
    Serial.println("running rotateScore");
    if (abs(target_pose.x - current_pose.x) < 10 && abs(target_pose.y - current_pose.y) < 10 && abs(target_pose.bearing - current_pose.bearing) < 1)
    {
        base.move(0, 0, 0);
        layer_1_rx_data.data.kick = true;
        robot.sendSerial();
    }
    else
    {
        target_pose.x = 910;
        target_pose.y = 1900;
        target_pose.bearing = 0;
        moveToTargetPose();
        layer_1_rx_data.data.kick = false;
    }
}

void Robot::moveToNeutralPoint(int neutral_point, bool behind_point)
{
    int neutral_points[5][2] = {{0, 0}, {0, 0}, {915, 1215}, {0, 0}, {0, 0}};
    target_pose.x = neutral_points[neutral_point][0];
    target_pose.y = behind_point ? neutral_points[neutral_point][1] - 200 : neutral_points[neutral_point][1];
    target_pose.bearing = 0;

    moveToTargetPose();
}