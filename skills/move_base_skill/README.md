# move_base_skill

## Skill summary

A skill for moving a robot's base between locations. The skill is a high-level interface to a robot's own navigation interface, namely target poses are sent to an action exposed by the robot's navigation stack.

The skill goal is defined so that three types of goal poses can be sent:
* `POSE`: The goal should be a list of poses of type `geometry_msgs/msg/PoseStamped`
* `NAMED_POSE`: The goal should be a list of named poses (i.e. strings that can be converted to `geometry_msgs/msg/PoseStamped`)
* `SEMANTIC_LOCATION`: The goal should be a list of semantic locations (similar to named poses, but this is meant for uses with semantic maps and locations such as rooms, where there isn't a one-to-one mapping between the location name and the actual pose)

**Note**: In the current version of the skill, only goals of type `POSE` are accepted.

## Execution description

The execution proceeds as follows:
1. The goals are sent to the robot's navigation component one by one
2. After each goal is sent, the skill waits until the navigation component sends completion feedback
3. Once the goal is reached (or the monitor times out), the next goal on the list is attempted

The execution can be cancelled at any time, such that cancelling the skill also cancels the ongoing navigation.