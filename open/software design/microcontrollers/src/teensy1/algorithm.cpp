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
        double offset, multiplier;

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
        // double speed = fmin(fmax(0.25, 0.00001 * pow(ball.distance_from_robot, 2)), 0.3);

        // TUNE THIS
        double orbit_min_speed = 0.1;
        double orbit_max_speed = 0.4;
        double orbit_decel_f = 65; // typically represents the maximum distance from the ball in pixels
        double orbit_decel_k = 0.065; // increase for faster deceleration
        // END TUNE

        // move slower when close to the ball
        
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
        double edge_a = 9;
        double edge_b = 0.03;
        double edge_c = -1.81;
        // END TUNE

        // scale the maximum speed based on the distance from the edge
        double orbit_max_speed_scaled = fmin(edge_a * exp((-edge_b * average_goal_x) + edge_c), orbit_max_speed);
        
        // deceleration curve
        // double speed = min(max(0.01 * ball.distance_from_robot, 0.15),  0.5);
        double speed = fmin(fmax(orbit_decel_k * exp(ball.distance_from_robot / orbit_decel_f), orbit_min_speed), orbit_max_speed_scaled);

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
            // Serial.print("goal_y_diff " + String(goal_y - ball.current_pose.y));
            // Serial.println(" goal_x_diff " + String(ball.current_pose.x - average_goal_x));
            // END TUNE

            if (abs(goal_y - ball.current_pose.y) < goal_y_diff_thresh && abs(goal_x - ball.current_pose.x) < goal_x_diff_thresh)
            {
                move_data.speed = 0;
                // digitalWrite(13, HIGH);
            }
            else
            {
                // digitalWrite(13, LOW);
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
        target_pose.x = blue_goal.current_pose.x;
        target_pose.y = blue_goal.current_pose.y + yellow_goal.current_pose.y;
        // Serial.println("x: " + String(target_pose.x) + " y: " + String(target_pose.y));
        goalieTrack();
        // move_data.speed = 0;
        // move_data.target_angle = 0;
        // move_data.target_bearing = 0;
        // move_data.ema_constant = 0.0002;

    }
}

void Robot::orbitScore()
{   
    // digitalWrite(13, HIGH);
    // Serial.println("running orbitScore");
    // double target_bearing = robot.dip_4_on ? yellow_goal.current_pose.bearing : blue_goal.current_pose.bearing;
    double goal_y = blue_goal.current_pose.y;
    double target_bearing;
    target_bearing = correctBearing(blue_open.current_pose.bearing + robot.current_pose.bearing);
    if (target_bearing > 180) {
        target_bearing = target_bearing - 360;
    }
    Serial.print(robot.current_pose.bearing);
    double accuracy_counter;

    // TUNE THIS
    double score_min_speed = 0.05;
    double score_max_speed = 0.25;
    double score_decel_f = 62;
    double score_decel_k = 0.05;

    double score_accel_time = 400;
    double score_steep_accel_time = 200;
    double score_turn_time = 300;
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

        //strafe and turn (without deceleration)
        float elapsed_duration = millis() - scoring_start_time;
        if (elapsed_duration < score_steep_accel_time)
        {
            move_data.speed = (elapsed_duration) / score_steep_accel_time * (score_max_speed - score_min_speed) + score_min_speed;
            move_data.target_angle = target_bearing - (elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing;
            move_data.target_bearing = (elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing;

        }
        else
        {
            if (elapsed_duration - score_steep_accel_time > 200) {
                robot.kicker.kick();
            }
            move_data.speed = score_max_speed;
            move_data.target_angle = 0;
            move_data.target_bearing = target_bearing;
        }
        Serial.println("bearing: " + String(target_bearing));

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