#ifndef COLLISION_SAFETY_STOP_UTILS_HPP
#define COLLISION_SAFETY_STOP_UTILS_HPP

#include <cmath>

struct CartesianPose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    static double get_position_norm(const CartesianPose &pose1, const CartesianPose &pose2)
    {
        return sqrt(pow(pose1.x - pose2.x, 2.0) +
                    pow(pose1.y - pose2.y, 2.0) +
                    pow(pose1.z - pose2.z, 2.0));
    }
};

#endif