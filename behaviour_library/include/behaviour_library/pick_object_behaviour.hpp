#ifndef PICK_OBJECT_BEHAVIOUR_HPP
#define PICK_OBJECT_BEHAVIOUR_HPP

#include <vector>
#include <behaviortree_ros2/bt_action_node.hpp>
#include "robot_swiss_knife_msgs/msg/object.hpp"
#include "pick_skill/action/pick_skill.hpp"

using namespace BT;

/**
 * A behaviour for interacting with an action of type pick_skill::action::PickSkill.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class PickObjectBehaviour : public RosActionNode<pick_skill::action::PickSkill>
{
public:
    PickObjectBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the action goal:
     * - Input ports:
     *     * pick_goal (pick_skill::action::PickSkill::Goal)
     */
    static PortsList providedPorts();

    /**
     * Sets the action goal based on the blackboard contents.
     */
    virtual bool setGoal(RosActionNode::Goal& goal) override;

    /**
     * Returns SUCCESS if the action's 'success' status is tre, returns FAILURE otherwise.
     */
    virtual NodeStatus onResultReceived(const WrappedResult& wr) override;

    /**
     * Registers a node failure.
     */
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    /**
     * Prints the received feedback from the action (if any), and returns RUNNING.
     */
    virtual NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

#endif