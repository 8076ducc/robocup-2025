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

Goal yellow_open;
Goal blue_open;

unsigned long last_time = 0;
int real = 0;
int points = 0;

// striker variables
unsigned long no_catchment_start_time = 0;
bool was_in_catchment = false;
const unsigned long catchment_timeout = 250; // ms

// goalie variables
unsigned long time_ball_stopped;
double ball_last_dist;
bool ball_approaching;

void striker()
{
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
  robot.task = 2; // running defendGoal
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

  // switch (robot.task)
  // {
  // case 0:
  //   // Serial.println("running task 0");
  //   // digitalWrite(13, HIGH);
  //   robot.orbitToBall(0);
  //   // robot.rotateToBall();
  //   break;

  // case 1:
  //   // digitalWrite(13, LOW);
  //   robot.orbitScore();
  //   break;

  // case 2:
  //   // digitalWrite(13, HIGH);
  //   robot.defendGoal();
  //   break;

  // case 3:
  //   kp = 0.0013;
  //   robot.moveToTargetPose();
  //   break;
  // }
  robot.defendGoal();
  goalie();
  // striker();
  // robot.orbitToBall(0);

  robot.base.move(robot.move_data.speed, robot.move_data.target_angle, robot.move_data.target_bearing, kp, ki, kd);
}
