#include "main.h"
#include <cstdlib>

Pose target_pose_wrt_goal;
Pose target_pose_wrt_ball;

unsigned long scoring_start_time;
unsigned long kicking_start_time;

unsigned long goalie_within_range_time;
double prev_ball_dist;

unsigned long not_approaching_start_time = 0;
bool was_approaching = false;
const unsigned long approaching_timeout = 200; // ms
const unsigned long goalie_timeout = 100; // ms

bool raffles = true;


double mapValue(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax) {
    // Formula for linear mapping
    return outputMin + (inputValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
}

void Robot::defendGoal()
{
    // double goal_y = blue_goal.current_pose.y;
    // double target_y_from_goal = -85;
    // target_pose.x = (abs(ball.current_pose.x) > 4) ? (ball.current_pose.x) : 0;
    // target_pose.x = bound(target_pose.x - blue_goal.current_pose.x, -55, 55) + blue_goal.current_pose.x;
    double ball_distance = sqrt(pow(ball.current_pose.x + robot.current_pose.x, 2) + pow(ball.current_pose.y + robot.current_pose.y, 2));
    double angle = ball.current_pose.bearing;

    if (angle > 180)
    {
        angle -= 360;
        angle = abs(angle);
    }

    if (line_data.on_line)
    {
        rejectLine(0);
    }
    else if (ball.current_pose.y > 0 && angle < 15) {
        if (robot.current_pose.y + ball.current_pose.y < 200) { // ball is behind the center line
            if (goalie_within_range_time == 0) {
                goalie_within_range_time = millis();
            }
            if (millis() - goalie_within_range_time > goalie_timeout)
            {
                if (!was_approaching) {
                    if (ball.distance_from_robot > 100){
                        goalieTrack();
                    } else {
                        not_approaching_start_time = millis(); // reset the timer
                        was_approaching = true;
                        goalieRush();
                    }
                } else {
                    not_approaching_start_time = millis(); // reset the timer
                    was_approaching = true;
                    goalieRush();
                }
            } else {
                goalieTrack();
            }
        } else {
            goalie_within_range_time = 0;
            goalieTrack();
        }
    }
    else
    {
        goalie_within_range_time = 0;
        goalieTrack();
    }
}

void Robot::orbitToBall(double bearing)
{
    if (ball.detected)
    {
        scoring_start_time = millis();
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
        // double b = 1.7; // pivots the curve - aggressiveness
        // double c = 150; // typically represents maximum distance from the ball
        // double d = 1; // maximum multiplier

        // TUNE THIS
        // double orbit_a = 0.21;
        // double orbit_b = 0.8;
        // double orbit_c = 2190;
        // double orbit_d = 1;

        double orbit_a = 0.17;
        double orbit_b = 1.2;
        double orbit_c = 2190;
        double orbit_d = 1;
        // END TUNE

        double factor = orbit_d - (ball.distance_from_robot) / orbit_c;
        double multiplier = fmin(orbit_d, orbit_a * exp(orbit_b * factor));

        // TUNE THIS
        double orbit_min_speed = 0.17;

        double orbit_slow_angle = 45;
        double orbit_slow_speed = 0.05;

        if (ball.current_pose.bearing > 360 - orbit_slow_angle || ball.current_pose.bearing < orbit_slow_angle)
        {
            orbit_min_speed = 0.09;
            // orbit_min_speed = mapValue(abs(principalise(ball.current_pose.bearing)), 30, orbit_slow_angle, orbit_min_speed, orbit_slow_speed);
            // orbit_min_speed = bound(orbit_min_speed, orbit_slow_speed, orbit_min_speed);
        }

        double orbit_max_speed = 0.3;
        double orbit_decel_f = 285;  // typically represents the maximum distance from the ball in pixels
        double orbit_decel_k = 0.06; // increase for faster deceleration
        // END TUNE


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
        double speed = fmin(fmax(orbit_decel_k * exp(ball.distance_from_robot / orbit_decel_f), orbit_min_speed), orbit_max_speed_scaled);

        double correction = correctBearing(bearing_from_robot + multiplier * offset);

        // if (correction < 10 || correction > 350)
        // {
        //     correction = 0;
        // }

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
            // END TUNE

            move_data.speed = speed;

            if (correction > 180 + 20 && correction < 360 - 20 && robot.current_pose.x < -600) {
                move_data.speed = 0;
            } else if (correction < 180 - 20 && correction > 20 && robot.current_pose.x > 600) {
                move_data.speed = 0;
            }

            move_data.target_angle = correction;
            move_data.target_bearing = bearing;
        }
    }
    else
    {
        if (raffles) {
            double max_speed;
            double min_speed;
            double bearing = 0;

            if (robot.current_pose.y < -280){
                if (robot.current_pose.y < -320) {
                    max_speed = 0.45;
                    min_speed = 0.35;
                } else {
                    max_speed = 0.2;
                    min_speed = 0.1;
                }
            } else {
                max_speed = 0.3;
                min_speed = 0.15;
            }
            if (ball.ball_last_seen.x > 0) {
                moveToPoint(500, -300, 0, min_speed, max_speed);
            } else {
                moveToPoint(-500, -300, 0, min_speed, max_speed);
            }
            
        } else {
            robot.moveToPoint(0, 0, 0, 0.15, 0.3);
        }
        
    }
}

int strategy = 0;

void Robot::orbitScore()
{
    // digitalWrite(13, HIGH);
    // Serial.println("running orbitScore");
    // double target_bearing = robot.dip_4_on ? yellow_goal.current_pose.bearing : blue_goal.current_pose.bearing;

    double target_bearing;
    target_bearing = yellow_open.current_pose.bearing;
    if (target_bearing > 180)
    {
        target_bearing = target_bearing - 360;
    }
    
    // END TUNE

    if (line_data.on_line)
    {
        rejectLine(target_bearing);
        // layer_1_rx_data.data.kick = true;
    }
    else
    {
        // if (strategy == 0) {
        //     int randomNum = rand() % 100;
        //     double strat_one_chance = map(robot.current_pose.y, 400, -1000, 100, 10);
        //     if (randomNum < strat_one_chance) {
        //         strategy = 1;
        //     } else {
        //         strategy = 2;
        //     }
        // }
        // if (strategy == 1)
        // {
        scoringStrategyOne();
        // }
        // else if (strategy == 2)
        // {
        //     scoringStrategyTwo();
        //     return 0.005;
        // }
        
    }
}

bool Robot::scoringStrategyOne() {
    double score_min_speed = 0.05;
    double score_max_speed = 0.4;

    double score_accel_time = 400;
    double score_steep_accel_time = 400;
    double score_turn_time = 300;

    double target_bearing;
    target_bearing = correctBearing(yellow_open.current_pose.bearing);
    if (target_bearing > 180)
    {
        target_bearing = target_bearing - 360;
    }

    // float elapsed_duration = millis() - scoring_start_time;
    // if (elapsed_duration < score_steep_accel_time)
    // {
    //     move_data.speed = (elapsed_duration) / score_steep_accel_time * (score_max_speed - score_min_speed) + score_min_speed;
    //     move_data.target_angle = target_bearing - (elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing;
    //     move_data.target_bearing = correctBearing((elapsed_duration - score_steep_accel_time) / score_turn_time * target_bearing);
    // }
    // else
    // {
        
    // }

    if (robot.current_pose.y > 200)
    {
        if (millis() - scoring_start_time < 300) {
            move_data.speed = 0.2;
        } else {
            move_data.speed = 0.05;
        }
        move_data.target_angle = 0;
        move_data.target_bearing = correctBearing(target_bearing);
    } else {
        move_data.speed = score_max_speed;
        move_data.target_angle = 0;
        move_data.target_bearing = correctBearing(target_bearing);
    }
}

void Robot::scoringStrategyTwo() {

    double score_min_speed = 0.25;
    double score_max_speed = 0.4;

    double move_point_min_speed = 0.05;
    double move_point_max_speed = 0.2;

    double move_point_accel_time = 400;
    double score_steep_accel_time = 50;

    double target_bearing;
    target_bearing = blue_open.current_pose.bearing;
    if (target_bearing > 180)
    {
        target_bearing = target_bearing - 360;
    }

    int direction = 0;

    if (robot.current_pose.y < 200) {
        float elapsed_duration = millis() - scoring_start_time;
        if (robot.current_pose.x > 0) {
            direction = 1;
        } else {
            direction = -1;
        }
        double speed = bound((elapsed_duration) / move_point_accel_time * (move_point_max_speed - move_point_min_speed) + move_point_min_speed, move_point_min_speed, move_point_max_speed);
        robot.moveToPoint(direction * 300, 300, 0, speed, speed);
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
                    strategy = 0;
                }
            }
            move_data.speed = score_max_speed;
            move_data.target_angle = 0;
            move_data.target_bearing = target_bearing;
        }
        
    }
}