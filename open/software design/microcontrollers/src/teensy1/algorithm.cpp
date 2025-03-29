#include "main.h"

Pose target_pose_wrt_goal;
Pose target_pose_wrt_ball;

unsigned long scoring_start_time;

void Robot::defendGoal()
{
    double goal_y = blue_goal.current_pose.y;
    double target_y_from_goal = -85;
    target_pose.x = (abs(ball.current_pose.x) > 4) ? (ball.current_pose.x) : 0;
    target_pose.x = bound(target_pose.x - blue_goal.current_pose.x, -55, 55) + blue_goal.current_pose.x;

    if (line_data.on_line)
    {
        rejectLine(0);
    }
    // else if (ball.current_pose.y < 0) // ball is behind the robot
    // {
    //     orbitToBall(0);
    // }
    else
    {
        target_pose.y = goal_y - target_y_from_goal;
        target_pose.bearing = 0;
        goalieTrack();
    }
}

void Robot::orbitToBall(double bearing)
{
    // Serial.println("running orbitToBall");
    if (ball.detected)
    {
        scoring_start_time = millis();
        // Serial.println("ball: " + String(ball.current_pose.bearing) + " robot(imu): " + String(robot.current_pose.bearing ));
        double bearing_from_robot = correctBearing(ball.current_pose.bearing - robot.current_pose.bearing);
        double offset;

        if (bearing_from_robot < 180)
        {
            offset = fmin(bearing_from_robot * 1.05, 90);
        }
        else
        {
            offset = fmax((bearing_from_robot - 360) * 1.05, -90);
        }
        // double a = 0.085; // affects orbit radius (shift in and out)
        // double b = 1.7; // pivots the curve
        // double c = 150; // typically represents maximum distance from the ball
        // double d = 1; // maximum multiplier

        // TUNE THIS
        double orbit_a = 0.17;
        double orbit_b = 0.8;
        double orbit_c = 2190;
        double orbit_d = 1;
        // END TUNE

        double factor = orbit_d - (ball.distance_from_robot) / orbit_c;
        double multiplier = fmin(orbit_d, orbit_a * exp(orbit_b * factor));

        // TUNE THIS
        double orbit_min_speed = 0.2;

        if (ball.current_pose.bearing > 360 - 55 || ball.current_pose.bearing < 55)
        {
            orbit_min_speed = 0.08;

        }

        double orbit_max_speed = 0.4;
        double orbit_decel_f = 355;  // typically represents the maximum distance from the ball in pixels
        double orbit_decel_k = 0.06; // increase for faster deceleration
        // END TUNE

        // SET ATTACKING GOAL
        double goal_y = blue_goal.current_pose.y;
        double goal_x = blue_goal.current_pose.x;
        // END SET ATTACKING GOAL

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
        double edge_a = 3.4;
        double edge_b = 0.007;
        double edge_c = 0.0;
        double edge_d = 0.2;
        // END TUNE

        // scale the maximum speed based on the distance from the edge
        double orbit_max_speed_scaled = fmin(edge_a * exp((-edge_b * average_goal_x) + edge_c) + edge_d, orbit_max_speed);

        // deceleration curve
        // double speed = min(max(0.01 * ball.distance_from_robot, 0.15),  0.5);
        double speed = fmin(fmax(orbit_decel_k * exp(ball.distance_from_robot / orbit_decel_f), orbit_min_speed), orbit_max_speed_scaled);
        // Serial.println(ball.distance_from_robot);
        // Serial.println("speed: " + String(speed));

        double correction = correctBearing(bearing_from_robot + multiplier * offset);
        // Serial.print("correction: ");
        // Serial.println(correction);

        if (correction < 10 || correction > 350)
        {
            correction = 0;
        }

        if (line_data.on_line)
        {
            if (abs(correction - line_data.initial_line_angle) < 90 && (line_data.initial_line_angle > 20 && line_data.initial_line_angle < 330))
            {
                rejectLine(bearing);
            }
            else
            {
                rejectLine(bearing);
            }
        }
        else
        {
            // TUNE THIS
            double goal_y_diff_thresh = 25;
            double goal_x_diff_thresh = 40;
            // Serial.print("goal_y_diff " + String(goal_y - ball.current_pose.y));
            // Serial.println(" goal_x_diff " + String(ball.current_pose.x - average_goal_x));
            // END TUNE

            if (abs(goal_y - ball.current_pose.y) < goal_y_diff_thresh && abs(goal_x - ball.current_pose.x) < goal_x_diff_thresh)
            {
                move_data.speed = 0;
            }
            else
            {
                move_data.speed = speed;
            }
            move_data.target_angle = correction;
            move_data.target_bearing = bearing;
        }
    }
    else
    {
        robot.moveToPoint(0, 0, 0, 0.15, 0.3);
    }
}

void Robot::orbitScore()
{
    // digitalWrite(13, HIGH);
    // Serial.println("running orbitScore");
    // double target_bearing = robot.dip_4_on ? yellow_goal.current_pose.bearing : blue_goal.current_pose.bearing;
    double goal_y = blue_goal.current_pose.y;
    double target_bearing;
    target_bearing = blue_open.current_pose.bearing;
    if (target_bearing > 180)
    {
        target_bearing = target_bearing - 360;
    }
    // Serial.print(robot.current_pose.bearing);
    double accuracy_counter;

    // TUNE THIS
    double score_min_speed = 0.05;
    double score_max_speed = 0.35;
    double score_decel_f = 62;
    double score_decel_k = 0.05;

    double score_accel_time = 400;
    double score_steep_accel_time = 100;
    double score_turn_time = 300;
    
    int direction = 0;
    // END TUNE

    if (line_data.on_line)
    {
        rejectLine(target_bearing);
        // layer_1_rx_data.data.kick = true;
    }
    else
    {
        // move_data.speed = min(bound(scoringcounter/1000, 0.05, 0.2) / cos(radians(bound(scoringcounter/1000, 0, 1) * target_bearing)), 0.4);// bound(score_decel_k * exp(goal_y / score_decel_f), score_min_speed, score_max_speed);

        // //strafe only (without deceleration)
        // float elapsed_duration = millis() - scoring_start_time;
        // if (elapsed_time < score_accel_time) {
        //     move_data.speed = (elapsed_time)/score_accel_time*(score_max_speed-score_min_speed)+score_min_speed;
        //     move_data.target_angle = (elapsed_time)/score_accel_time*target_bearing;
        // } else {
        //     move_data.speed = score_max_speed;
        //     move_data.target_angle = target_bearing;
        // }

        // strat 1 (without decel)
        // float elapsed_duration = millis() - scoring_start_time;
        // if (elapsed_duration < score_steep_accel_time)
        // {
        //     move_data.speed = (elapsed_duration) / score_steep_accel_time * (score_max_speed - score_min_speed) + score_min_speed;
        //     move_data.target_angle = target_bearing - (elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing;
        //     move_data.target_bearing = (elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing;
        // }
        // else
        // {
        //     if (elapsed_duration - score_steep_accel_time > 200)
        //     {
        //         robot.kicker.kick();
        //     }
        //     move_data.speed = score_max_speed;
        //     move_data.target_angle = 0;
        //     move_data.target_bearing = target_bearing;
        // }

        //strat 2 (without decel)
        unsigned long kicking_start_time = 0;
        // Serial.println("y: " + String(robot.current_pose.y));
        if (robot.current_pose.y < 0) {
            float elapsed_duration  = millis() - scoring_start_time;
            // if (elapsed_duration < 2) {
                if (robot.current_pose.x > 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                // Serial.println("set direction" + String(direction));
            // }
            // Serial.println("direction: " + String(direction));
            robot.moveToPoint(direction * 500, 400, 0, bound((elapsed_duration) / 300 * (0.35 - 0.2) + 0.2, 0.2, 0.25), bound((elapsed_duration) / 300 * (0.35 - 0.2) + 0.2, 0.2, 0.25));
            kicking_start_time = millis();
        } else {
            direction=0;
            float elapsed_duration = millis() - kicking_start_time;
            if (elapsed_duration < score_steep_accel_time)
            {
                move_data.speed = score_max_speed;
                move_data.target_angle = target_bearing - (elapsed_duration) / score_steep_accel_time * target_bearing;
                move_data.target_bearing = (elapsed_duration) / score_steep_accel_time * target_bearing;
            }
            else
            {   
                if (elapsed_duration - score_steep_accel_time > 200)
                {
                    if (robot.current_pose.bearing - blue_open.current_pose.bearing < 5)
                    {
                        robot.kicker.kick();
                    }
                }
                move_data.speed = score_max_speed;
                move_data.target_angle = 0;
                move_data.target_bearing = target_bearing;
            }
            
        }
        // } else {
        //     float elapsed_duration = millis() - kicking_start_time;
        //     if (elapsed_duration < score_steep_accel_time)
        //     {
        //         move_data.speed = (elapsed_duration) / score_steep_accel_time * (score_max_speed - score_min_speed) + score_min_speed;
        //         move_data.target_angle = target_bearing - (elapsed_duration) / score_steep_accel_time * target_bearing;
        //         move_data.target_bearing = (elapsed_duration) / score_steep_accel_time * target_bearing;
        //     }
        //     else
        //     {   
        //         if (elapsed_duration - score_steep_accel_time > 200)
        //         {
        //             if (robot.current_pose.bearing - blue_open.current_pose.bearing < 5)
        //             {
        //                 robot.kicker.kick();
        //             }
        //         }
        //         move_data.speed = score_max_speed;
        //         move_data.target_angle = 0;
        //         move_data.target_bearing = target_bearing;
        //     }
        // }
        

        // if (elapsed_duration < score_accel_time)
        // {
        //     move_data.speed = (elapsed_duration) / score_accel_time * (score_max_speed - score_min_speed) + score_min_speed;
        //     move_data.target_angle = (elapsed_duration) / score_accel_time * target_bearing;
        //     move_data.target_bearing = 0;

        // }
        // else if (elapsed_duration - score_accel_time < score_turn_time)
        // {
        //     move_data.speed = bound(score_decel_k * exp(goal_y / score_decel_f), score_min_speed, score_max_speed);
        //     move_data.target_angle = target_bearing - (elapsed_duration - score_accel_time) / score_turn_time * target_bearing;
        //     move_data.target_bearing = (elapsed_duration - score_accel_time) / score_turn_time * target_bearing;
        // }
        // else
        // {
        //     move_data.speed = bound(score_decel_k * exp(goal_y / score_decel_f), score_min_speed, score_max_speed);
        //     move_data.target_angle = 0;
        //     move_data.target_bearing = target_bearing;
        // }

        // if(move_data.target_angle != 0) {
        //     Serial.println("angle: " + String(move_data.target_angle) + " bearing: " + String(move_data.target_bearing));
        // }

        // else if (elapsed_duration - score_accel_time < score_turn_time)
        // if (move_data.speed == score_max_speed) {
        //     move_data.speed = bound(score_decel_k * exp(goal_y / score_decel_f), score_min_speed, score_max_speed);
        //     if (target_bearing < 0) {
        //         move_data.target_bearing = bound(-(scoring_counter - score_max_speed*1000)/8, target_bearing, 0);
        //         move_data.target_angle = target_bearing - bound(-(scoring_counter - score_max_speed*1000)/8, target_bearing, 0);
        //     } else {
        //         move_data.target_bearing = bound((scoring_counter - score_max_speed*1000)/8, 0, target_bearing);
        //         move_data.target_angle = target_bearing - bound((scoring_counter - score_max_speed*1000)/8, 0, target_bearing);
        // }
        // } else {
        // }
        // move_data.target_angle = bound(scoringcounter/2500, 0, 1) * target_bearing;
        // move_data.speed = bound(move_data.speed, score_min_speed, score_max_speed);
        move_data.ema_constant = 0.00017;
    }
}