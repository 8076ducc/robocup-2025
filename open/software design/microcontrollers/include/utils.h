#pragma once

#ifndef UTILS_H
#define UTILS_H

class Pose
{
public:
    double x;
    double y;
    double bearing;
};

struct Ball
{
    Pose current_pose;
    Pose projected_pose;

    Pose ball_last_seen;

    double distance_from_robot;

    bool in_catchment;
    bool in_alliance_catchment;
    bool detected;
};

struct Goal
{
    Pose current_pose;
    
    bool detected;
};

#endif