#include "main.h"
#include "HardwareSerial.h"
#include <TeensyThreads.h>

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
const unsigned long catchment_timeout = 400; // ms

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

volatile int count = 0;

void resetThread()
{
  while (true)
  {
    robot.kicker.reset();
    threads.delay(1);
  }
}

// void serialThread()
// {
//   while (1)
//   {

//     count += 1;
//     threads.delay(1);
//   }
// }

// void loopThread()
// {
//   while (true)
//   {
//     robot.sendSerial();
//     robot.updateSerial();
//     // double flick_angle = 0;
//     double kp = 0.0014; // orginally 0.0014
//     double ki = 0.0;    // orginally 0.0
//     double kd = 0.005;  // orginally 0.005

// #ifndef BOT_A
//     // pure goalie code
//     if (!robot.alliance_robot_detected)
//     {
//       if (robot.dip_1_on)
//       {
//         striker();
//       }
//       else
//       {
//         goalie();
//       }
//     }
//     else
//     {
//       if (robot.dip_2_on)
//       {
//         striker();
//       }
//       else
//       {
//         robot.task = 2;
//       }
//     }

// #else
//     // pure striker code
//     // striker();
//     // if (!robot.alliance_robot_detected && ball.current_pose.y < 700)
//     // {
//     //   goalie();
//     // }
//     // else
//     // {
//     //   striker();
//     // }
// #endif

//     // Serial.print("task: ");x
//     // Serial.println(robot.task);

//     switch (robot.task)
//     {
//     case 0:
//       // Serial.println("running task 0");
//       // digitalWrite(13, HIGH);
//       robot.orbitToBall(0);
//       // robot.rotateToBall();
//       break;

//     case 1:
//       // digitalWrite(13, LOW);
//       robot.orbitScore();
//       break;

//     case 2:
//       // digitalWrite(13, HIGH);
//       robot.defendGoal();
//       break;

//     case 3:
//       kp = 0.0013;
//       // robot.moveToTargetPose();
//       break;
//     }
//     // goalie();
//     // Serial.print("blue x: " + String(blue_goal.current_pose.x));

//     striker();

//     // Serial.println("ball: " + String(ball.in_catchment));
//     // digitalWrite(13, HIGH);
//     // robot.kicker.kick();
//     // robot.kicker.reset();
//     // robot.orbitToBall(0);

//     // if (abs(ball.current_pose.x) > 6){
//     //   digitalWrite(13, HIGH);
//     // } else {
//     //   digitalWrite(13, LOW);
//     // }
//     // moveToCenter();÷

//     // robot.target_pose.x = blue_goal.current_pose.x;
//     // robot.target_pose.y = blue_goal.current_pose.y + yellow_goal.current_pose.y;
//     // Serial.println("x: " + String(robot.target_pose.x) + " y: " + String(robot.target_pose.y));
//     // robot.goalieTrack();
//     // Serial.println("angle: " + String(robot.move_data.target_angle) + "bearing: " + String(robot.move_data.target_bearing));

//     robot.base.move(robot.move_data.speed, robot.move_data.target_angle, robot.move_data.target_bearing, kp, ki, kd);
//     threads.delay(1);
//   }
// }

// int enter_sleep(int ms) {
//   delay(ms);//set sleep time in milliseconds
//   //additional, one can use the RTC or a low power timer
//   //to calculate actual time spent asleep and return the
//   //value to the scheduler to calculate better times.
//   return ms;
// }

void setup()
{
  robot.base.setUp();
  robot.setUpSerial();

  robot.previous_pose.x = 0;
  robot.previous_pose.y = 0;
  robot.previous_pose.bearing = 0;

  last_time = millis();
  robot.dribbler.update();
  robot.dribbler.dribbling = true;

  robot.kicker.setup();

  pinMode(13, OUTPUT);
  pinMode(23, OUTPUT);

  threads.addThread(resetThread, 0, 10000);

  // threads.addThread(loopThread);
  // reset.detach();
  // main.detach();
  // serial.detach();
}

long unsigned timer = 0;
// int count = 0;

void loop()
{
  // Serial.print("0. : " + String(micros() - timer));
  // timer = micros();
  robot.sendSerial();
  // Serial.println("1.: " + String(micros() - timer));
  // timer = micros();
  robot.updateSerial();
  // Serial.println("2.: " + String(micros() - timer));
  // timer = micros();
  robot.storeRobotPose();
  // Serial.println("3.: " + String(micros() - timer));
  // timer = micros();

  double kp = 0.0014; // orginally 0.0014
  double ki = 0.0;    // orginally 0.0
  double kd = 0.005;  // orginally 0.005

  switch (robot.task)
  {
  case 0:
    // Serial.println("running task 0");
    // digitalWrite(13, HIGH);
    robot.orbitToBall(0);
    // robot.rotateToBall();
    break;

  case 1:
    // digitalWrite(13, LOW);
    robot.orbitScore();
    kp = 0.005;
    break;

  case 2:
    // digitalWrite(13, HIGH);
    robot.defendGoal();
    break;

  case 3:
    kp = 0.0013;
    // robot.moveToTargetPose();
    break;
  }
  // Serial.println("4.: " + String(micros() - timer));
  // timer = micros();
  // goalie();
  // Serial.println("ball in catchment: " + String(ball.in_catchment));
  striker();
  // Serial.println("5.: " + String(micros() - timer));
  // timer = micros();

  // if (millis() - timer > 1000) {
  //   timer = millis();
  //   count ++ ;
  // } else {
    // robot.moveToPoint(0, 0, 0);
  // }

  // Serial.println("x: " + String(robot.current_pose.x) + ", y: " + String(robot.current_pose.y) + " bearing: " + String(robot.current_pose.bearing));
  // Serial.println("blue x: " + String(blue_goal.current_pose.x) + ", goal y: " + String(blue_goal.current_pose.y));
  // Serial.println("yellow x: " + String(yellow_goal.current_pose.x) + ", yellow y: " + String(yellow_goal.current_pose.y));

  // robot.orbitToBall(0);

  robot.base.move(robot.move_data.speed, robot.move_data.target_angle, robot.move_data.target_bearing, kp, ki, kd);

  // Serial.println("6.: " + String(micros() - timer));
  // timer = micros();
  // Serial.println("target: " + String(robot.move_data.target_bearing) + "actual: " + String(robot.current_pose.bearing));
  delayMicroseconds(1);
}
