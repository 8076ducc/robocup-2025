#include "main.h"
#include "HardwareSerial.h"

PacketSerial Layer1Serial;
PacketSerial ImuSerial;
PacketSerial TeensySerial;

Layer1RxDataUnion layer_1_rx_data;
ImuRxDataUnion imu_rx_data;
Teensy1TxDataUnion teensy_1_tx_data;

Robot robot;
Ball ball;
Goal yellow_goal;
Goal blue_goal;

unsigned long last_time = 0;
int real = 0;
int points = 0;

int ball_catchment = 0;

// #define DEBUG

unsigned long time_ball_stopped;
double ball_last_dist;
bool was_goalie_now_striker;
bool is_goalie;
bool ball_approaching;

void striker()
{
  static unsigned long no_catchment_start_time = 0; // stores the time when ball leaves catchment
  static bool was_in_catchment = false; // tracks if the ball was previously in catchment
  const unsigned long catchment_timeout = 500; // in milliseconds
  if (ball.in_catchment == 1)
  {
    no_catchment_start_time = millis(); // reset the timer
    was_in_catchment = true;
    robot.task = 1; // running orbitScore
  }
  else if (was_in_catchment) // if ball was previously in catchment, start timing
  {
    if (millis() - no_catchment_start_time > catchment_timeout) // check elapsed time
    {
      robot.task = 0; // running orbitToBall
      was_in_catchment = false;
    }
    else
    {
      robot.task = 1; // continue running orbitScore
    }
  }
  else
  {
    robot.task = 0; // running orbitToBall
  }
}


void goalie()
{
  if (ball_last_dist - ball.distance_from_robot > 1)
  {
    ball_approaching = true;
    time_ball_stopped = millis();
  } else {
    ball_approaching = false;
  }

  ball_last_dist = ball.distance_from_robot;
  // Serial.print("time ball stopped: ");
  // Serial.println(time_ball_stopped);

  // simplify this to run without clock -> TEMPORARY
  // if (millis() - time_ball_stopped > 500 && ball.distance_from_robot < 100 && !was_goalie_now_striker)
  if ((ball_approaching == true && ball.distance_from_robot < 100) || (millis() - 100 < time_ball_stopped))
  {
    // Serial.println("intercept the ball/enemy striker");
    // robot.task = 0;
    was_goalie_now_striker = true;
  }
  // else if (ball.detected && ball.current_pose.y < 0)
  // {
  //   robot.task = 0;
  //   was_goalie_now_striker = true;
  // }

  //uncommented because idk what it does and i havent asked isaac/haojun
  // else if (was_goalie_now_striker && robot.current_pose.y < 700)
  // {
  //   robot.task = 0;
  // }
  else
  {
    // Serial.println("defend goal");
    was_goalie_now_striker = false;
  }
  robot.task = 2;
}

void setup()
{
  robot.base.setUp();
  robot.setUpSerial();
  pinModeFast(DRIBBLER_PWM, OUTPUT);
  pinModeFast(LIDAR_PWM, OUTPUT);

  robot.previous_pose.x = 0;
  robot.previous_pose.y = 0;
  robot.previous_pose.bearing = 0;

  last_time = millis();
  robot.dribbler.update();
  robot.dribbler.dribbling = true;

  
  pinMode(13, OUTPUT);
}

void loop()
{
  robot.updateSerial();
  robot.sendSerial();

  // double flick_angle = 0;
  double kp = 0.0014; // orginally 0.0014
  double ki = 0.0; // orginally 0.0
  double kd = 0.005; // orginally 0.005

#ifndef BOT_A
  // pure goalie code
  if (!robot.alliance_robot_detected)
  {
    if (robot.dip_1_on)
    {
      striker();
    }
    else
    {
      goalie();
    }
  }
  else
  {
    if (robot.dip_2_on)
    {
      striker();
    }
    else
    {
      robot.task = 2;
    }
  }

#else
  // pure striker code
  // striker();
  // if (!robot.alliance_robot_detected && ball.current_pose.y < 700)
  // {
  //   goalie();
  // }
  // else
  // {
  //   striker();
  // }
#endif

  // Serial.print("task: ");
  // Serial.println(robot.task);

  switch (robot.task)
  {
  case 0:
    Serial.println("running task 0");
    digitalWrite(13, LOW);
    robot.orbitToBall(0);
    // robot.rotateToBall();
    break;
  case 1:
    // robot.move_data.speed = 0;
    // robot.move_data.target_bearing = flick_angle;
    // robot.move_data.target_angle = 0;
    robot.orbitScore();
    break;
  case 2:
    // Serial.println("running defendGoal");
    digitalWrite(13, HIGH);
    robot.defendGoal();
    break;
  case 3:
    kp = 0.0013;
    robot.moveToTargetPose();
    break;
  }

  goalie();
  // robot.orbitToBall(0);

  robot.base.move(robot.move_data.speed, robot.move_data.target_angle, robot.move_data.target_bearing, kp, ki, kd);
}
