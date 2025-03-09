#include "main.h"

void onLayer1Received(const byte *buf, size_t size)
{
    Serial.println("Received data from L1");
    Layer1TxDataUnion data_received;

    // // Don't continue if the payload is invalid
    // if (size != sizeof(data_received))
    // {
    //     Serial.println("Invalid payload size from L1. Expected: " + String(sizeof(data_received)) + " Received: " + String(size));
    //     return;
    // }

    std::copy(buf, buf + size, std::begin(data_received.bytes));

    robot.line_data.on_line = data_received.data.on_line;

    if (robot.line_data.on_line)
    {
        robot.line_data.line_angle = data_received.data.line_angle;
        if (!robot.line_data.already_on_line)
        {
            robot.line_data.initial_line_angle = robot.line_data.line_angle;
            robot.line_data.already_on_line = true;
        }
    }
    else
    {
        robot.line_data.line_angle = 0;
        robot.line_data.initial_line_angle = 0;
        robot.line_data.already_on_line = false;
    }

    ball.in_catchment = data_received.data.ball_in_catchment;
    robot.line_data.line_centre = data_received.data.line_centre;
    robot.line_data.chord_length = data_received.data.chord_length;
    robot.line_data.line_start_ldr = data_received.data.line_start_ldr;
    robot.line_data.line_end_ldr = data_received.data.line_end_ldr;

    // Serial.println("ball in catchment: " + String(ball.in_catchment));
    // Serial.println("Line data:");
    // Serial.print("On line: ");
    // Serial.println(robot.line_data.on_line);
}

void onImuReceived(const byte *buf, size_t size)
{
    // Serial.println("Received data from IMU");
    ImuTxDataUnion data_received;

    // Don't continue if the payload is invalid
    if (size != sizeof(data_received))
    {
        // Serial.print("Invalid payload size from IMU. Expected: ");
        // Serial.print(sizeof(data_received));
        // Serial.print(" Received: ");
        // Serial.println(size);
        return;
    }

    std::copy(buf, buf + size, std::begin(data_received.bytes));

    teensy_1_tx_data.data.bearing = data_received.data.bearing;
    Serial.println("Bearing (IMU data): " + String(teensy_1_tx_data.data.bearing));
    robot.current_pose.bearing = data_received.data.bearing;

    robot.sendSerial();
}

void onTeensyReceived(const byte *buf, size_t size)
{
    CamTxDataUnion data_received;

    // Serial.println("Received data");
    // Serial.println(size);

    // // Don't continue if the payload is invalid
    // if (size != sizeof(data_received)) {
    //     Serial.print("Invalid payload size from RPI. Expected: " + String(sizeof(data_received)) + " Received: " + String(size));
    //     return;
    // }

    std::copy(buf, buf + size, std::begin(data_received.bytes));

    // Serial.print("Yellow goal: ");
    // Serial.print(data_received.data.yellow_goal_detected);
    // Serial.print(" ");
    // Serial.print(data_received.data.yellow_goal_x);
    // Serial.print(" ");
    // Serial.print(data_received.data.yellow_goal_y);
    // Serial.print(" ");

    // Serial.print(" Blue goal: ");
    // Serial.print(data_received.data.blue_goal_detected);
    // Serial.print(" ");
    // Serial.print(data_received.data.blue_goal_x);
    // Serial.print(" ");
    // Serial.print(data_received.data.blue_goal_y);
    // Serial.print(" ");
    
    // Serial.print(" Ball: ");
    // Serial.print(data_received.data.ball_detected);
    // Serial.print(" ");
    // Serial.print(data_received.data.ball_x);
    // Serial.print(" ");
    // Serial.print(data_received.data.ball_y);
    
    if (data_received.data.yellow_goal_detected && data_received.data.blue_goal_detected)
    {
        // Serial.println("Both goals detected");
        robot.storeCameraPose(data_received.data.yellow_goal_x, data_received.data.yellow_goal_y, data_received.data.blue_goal_x, data_received.data.blue_goal_y);
    }
    else if (data_received.data.yellow_goal_detected)
    {
        // Serial.println("Yellow goal detected");
        robot.storeSingleCameraPose(data_received.data.yellow_goal_x, data_received.data.yellow_goal_y);
    }
    else if (data_received.data.blue_goal_detected)
    {
        // Serial.println("Blue goal detected");
        robot.storeSingleCameraPose(data_received.data.blue_goal_x, data_received.data.blue_goal_y);
    }

    if (data_received.data.ball_detected)
    {
        double ball_relative_bearing = degrees(atan2(data_received.data.ball_x, data_received.data.ball_y));

        ball.current_pose.bearing = correctBearing(ball_relative_bearing + robot.current_pose.bearing);
        // ball.current_pose.x = bound(robot.current_pose.x + data_received.data.ball_x, 0, 1580);
        // ball.current_pose.y = bound(robot.current_pose.y + data_received.data.ball_y, 0, 2190);
        ball.current_pose.x = data_received.data.ball_x;
        ball.current_pose.y = data_received.data.ball_y;
        ball.detected = true;

    }
    else
    {
        ball.detected = false;
    }
    // // Serial.println("Received data from Teensy2");
    // Teensy1RxDataUnion data_received;

    // // Don't continue if the payload is invalid
    // if (size != sizeof(data_received))
    // {
    //     // Serial.print("Invalid payload size from Teensy2. Expected: ");
    //     // Serial.print(sizeof(data_received));
    //     // Serial.print(" Received: ");
    //     // Serial.println(size);
    //     // return;
    // }

    // std::copy(buf, buf + size, std::begin(data_received.bytes));

    // robot.current_pose = data_received.data.current_pose;
    // // robot.target_pose = data_received.data.target_pose;

    // // ball.current_pose = robot.target_pose;
    // ball.current_pose = data_received.data.target_pose;
    // // ball.current_pose.bearing -= robot.current_pose.bearing;

    // // Serial.println(robot.current_pose.bearing);
    // // ball.current_pose.bearing = correctBearing(ball.current_pose.bearing);
    // ball.detected = data_received.data.ball_detected;

    // robot.alliance_robot_detected = data_received.data.robot_detected;

    // // Serial.print("robot detected: ");
    // // Serial.println(robot.alliance_robot_detected);

    // // Serial.print("Robot: ");
    // // Serial.print(robot.current_pose.x);
    // // Serial.print(" ");
    // // Serial.print(robot.current_pose.y);
    // // Serial.print(" ");
    // // Serial.println(robot.current_pose.bearing);
    
    // // Serial.print(" Ball: ");
    // // Serial.print(ball.current_pose.x);
    // // Serial.print(" ");
    // // Serial.print(ball.current_pose.y);
    // // Serial.print(" ");
    // // Serial.println(ball.current_pose.bearing);

    // // ball.distance_from_robot = sqrt(pow(data_received.data.target_pose.x - data_received.data.current_pose.x, 2) + pow(data_received.data.target_pose.y - data_received.data.current_pose.y, 2));
    // ball.distance_from_robot = sqrt(pow(data_received.data.target_pose.x, 2) + pow(data_received.data.target_pose.y, 2));

    // // Serial.print(" ");
    // // Serial.print(ball.current_pose.bearing);
    // // Serial.print(" ");
    // //Serial.println(ball.distance_from_robot);

    // yellow_goal.current_pose.bearing = data_received.data.yellow_goal.current_pose.bearing;
    // blue_goal.current_pose.bearing = data_received.data.blue_goal.current_pose.bearing;

    // robot.dip_1_on = data_received.data.dip_1_on;
    // robot.dip_2_on = data_received.data.dip_2_on;
    // robot.dip_3_on = data_received.data.dip_3_on;
    // robot.dip_4_on = data_received.data.dip_4_on;
}

void Robot::setUpSerial()
{
#ifdef SERIAL_DEBUG
    Serial.begin(serial_monitor_baud);
    while (!Serial)
    {
    }
    Serial.println("Debug serial connection established.");
#endif

    Serial1.begin(layer_1_serial_baud);
    while (!Serial1)
    {
    }
    Layer1Serial.setStream(&Serial1);
    Layer1Serial.setPacketHandler(&onLayer1Received);
#ifdef SERIAL_DEBUG
    Serial.println("Layer 1 serial connection established.");
#endif

    Serial3.begin(imu_serial_baud);
    while (!Serial3)
    {
    }
    ImuSerial.setStream(&Serial3);
    ImuSerial.setPacketHandler(&onImuReceived);
#ifdef SERIAL_DEBUG
    Serial.println("IMU serial connection established.");
#endif

    Serial5.begin(teensy1_serial_baud);
    while (!Serial5)
    {
    }
    TeensySerial.setStream(&Serial5);
    TeensySerial.setPacketHandler(&onTeensyReceived);
#ifdef SERIAL_DEBUG
    Serial.println("Teensy serial connection established.");
#endif
}

void Robot::updateSerial()
{
    Layer1Serial.update();
    ImuSerial.update();
    TeensySerial.update();
}

void Robot::sendSerial()
{
    if (Serial1.availableForWrite())
    {
        Layer1Serial.send(layer_1_rx_data.bytes, sizeof(layer_1_rx_data.bytes));
    }

    if (Serial5.availableForWrite())
    {
        TeensySerial.send(teensy_1_tx_data.bytes, sizeof(teensy_1_tx_data.bytes));
    }
}