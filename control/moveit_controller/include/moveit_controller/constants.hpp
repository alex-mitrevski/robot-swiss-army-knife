#ifndef MOVEIT_CARTESIAN_CONTROLLER_CONSTANTS_HPP
#define MOVEIT_CARTESIAN_CONTROLLER_CONSTANTS_HPP

#include <string>
#include <vector>
#include <map>

enum GoalType {WAYPOINTS, NAMED_POSE};

std::map<std::string, std::vector<double>> NAMED_POSES =
{
    {"home", {0.25, 0.5, -1.34, -0.48, 1.94, -1.5, 1.37, 0.0}},
    {"neutral", {0.23, 1.3, 1.0, 0.0, 1.0, -1.5, 1.3, 0.0}},
    {"pregrasp", {0.16, 1.8, 1.0, 0.9, 1.42, -1.13, 1.37, -0.68}},
    {"pregrasp_side", {0.25, 1.46, -0.81, -3.22, 2.35, -1.67, -1.4, 0.02}}
};

#endif

