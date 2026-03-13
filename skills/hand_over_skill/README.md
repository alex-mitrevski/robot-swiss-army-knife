# hand_over_skill

## Skill summary

A ROS2 skill using which a robot hands an object over to a person.

## Dependencies

The robot depends on the `robot_swiss_knife_msgs` package.

## Execution description

The execution proceeds as follows:
1. If anthropometric parameter estimation is enabled, the robot estimates the parameters so that it can adjust the hand-over position accordingly.
2. The robot moves to a hand-over position.
3. Finally, the robot waits for a person to take the person (if the object is not picked up by a person, the robot gives up). The hand-over detection is performed by monitoring the force measurements.
