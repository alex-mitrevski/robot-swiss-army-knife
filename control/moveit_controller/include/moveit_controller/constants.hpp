#ifndef MOVEIT_CARTESIAN_CONTROLLER_CONSTANTS_HPP
#define MOVEIT_CARTESIAN_CONTROLLER_CONSTANTS_HPP

#include <string>
#include <vector>
#include <map>

enum GoalType {WAYPOINTS, NAMED_POSE};

std::map<std::string, std::vector<double>> NAMED_POSES =
{
    {"home", {0.23, 1.3, 1.0, 0.0, 1.0, -1.5, 1.3, 0.0}},
    {"pregrasp", {0.16, 1.8, 1.0, 0.9, 1.4, -1.1, 1.4, -0.68}}
};

#endif