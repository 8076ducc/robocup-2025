#include "main.h"

double regressBall(double distance)
{
    double regressed_distance = ((0 * pow(distance, 5)) - (0 * pow(distance, 4)) + (0.0002562291 * pow(distance, 3)) - (0.2172749012 * pow(distance, 2)) + (62.9773907121 * distance) - 5797.8985126998);
    return regressed_distance;
}

void onLayer1Received(const byte *buf, size_t size)
{
    // // Serial.println("Received data from L1");
    Layer1TxDataUnion data_received;

    // Don't continue if the payload is invalid
    if (size != sizeof(data_received))
    {
        // Serial.println("Invalid payload size from L1. Expected: " + String(sizeof(data_received)) + " Received: " + String(size));
        return;
    }

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
    // Serial.println("Bearing (IMU data): " + String(teensy_1_tx_data.data.bearing));
    robot.current_pose.bearing = data_received.data.bearing;

    robot.sendSerial();
}

// unsigned long last_serial_time = 0;

void onTeensyReceived(const byte *buf, size_t size) // receives shit from the camera
{
    CamTxDataUnion data_received;
    // Serial.println(size);
    // Serial.println(micros() - last_serial_time);
    // last_serial_time = micros()

    // Don't continue if the payload is invalid
    if (size != sizeof(data_received))
    {
        Serial.print("Invalid payload size from RPI. Expected: " + String(sizeof(data_received)) + " Received: " + String(size));
        // digitalWrite(13, HIGH);
        return;
    }

    // digitalWrite(13, LOW);

    std::copy(buf, buf + size, std::begin(data_received.bytes));

    yellow_goal.detected = data_received.data.yellow_goal_detected;
    blue_goal.detected = data_received.data.blue_goal_detected;

    // Serial.print("Yellow goal: ");
    // Serial.print(data_received.data.yellow_goal_detected);
    // Serial.print(" ");
    // Serial.print(data_received.data.yellow_goal_x);
    // Serial.print(" ");
    // Serial.print(data_received.data.yellow_goal_y);
    // Serial.println(" ");

    // Serial.print(" Blue goal: ");
    // Serial.print(data_received.data.blue_goal_detected);
    // Serial.print(" ");
    // Serial.print(data_received.data.blue_goal_x);
    // Serial.print(" ");
    // Serial.print(data_received.data.blue_goal_y);
    // Serial.println(" ");

    Serial.print(" Ball: ");
    Serial.print(data_received.data.ball_detected);
    Serial.print(" ");
    Serial.print(data_received.data.ball_x);
    Serial.print(" ");
    Serial.println(data_received.data.ball_y);

    if (yellow_goal.detected && blue_goal.detected)
    {
        // Serial.println("Both goals detected");
        robot.storeGoalPose(data_received.data.yellow_goal_x, data_received.data.yellow_goal_y, data_received.data.blue_goal_x, data_received.data.blue_goal_y);
        robot.storeGoalOpenPose(data_received.data.yellow_open_x, data_received.data.yellow_open_y, data_received.data.blue_open_x, data_received.data.blue_open_y);
    }
    else if (yellow_goal.detected)
    {
        // Serial.println("Yellow goal detected");
        robot.storeYellowPose(data_received.data.yellow_goal_x, data_received.data.yellow_goal_y);
        robot.storeYellowOpenPose(data_received.data.yellow_open_x, data_received.data.yellow_open_y);
    }
    else if (blue_goal.detected)
    {
        // Serial.println("Blue goal detected");
        robot.storeBluePose(data_received.data.blue_goal_x, data_received.data.blue_goal_y);
        robot.storeBlueOpenPose(data_received.data.blue_open_x, data_received.data.blue_open_y);
    }

    if (data_received.data.ball_detected)
    {
        double ball_relative_bearing = degrees(atan2(data_received.data.ball_x, data_received.data.ball_y));

        ball.current_pose.bearing = correctBearing(ball_relative_bearing + robot.current_pose.bearing);
        // ball.current_pose.x = bound(robot.current_pose.x + data_received.data.ball_x, 0, 1580);
        // ball.current_pose.y = bound(robot.current_pose.y + data_received.data.ball_y, 0, 2190);

        ball.detected = true;

        ball.distance_from_robot = sqrt(pow(data_received.data.ball_x*2, 2) + pow(data_received.data.ball_y*2, 2));
        ball.distance_from_robot = regressBall(ball.distance_from_robot);

        // ball.current_pose.x = sin(radians(ball_relative_bearing)) * ball.distance_from_robot;
        // ball.current_pose.y = cos(radians(ball_relative_bearing)) * ball.distance_from_robot;
        ball.current_pose.x = data_received.data.ball_x;
        ball.current_pose.y = data_received.data.ball_y;
    }
    else
    {
        ball.detected = false;
    }
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
    // Serial.println("Layer 1 serial connection established.");
#endif

    Serial3.begin(imu_serial_baud);
    while (!Serial3)
    {
    }
    ImuSerial.setStream(&Serial3);
    ImuSerial.setPacketHandler(&onImuReceived);
#ifdef SERIAL_DEBUG
    // Serial.println("IMU serial connection established.");
#endif

    Serial5.begin(teensy1_serial_baud);
    while (!Serial5)
    {
    }
    TeensySerial.setStream(&Serial5);
    TeensySerial.setPacketHandler(&onTeensyReceived);
#ifdef SERIAL_DEBUG
    // Serial.println("Teensy serial connection established.");
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