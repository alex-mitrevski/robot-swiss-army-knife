#ifndef MOVE_BASE_BEHAVIOUR_HPP
#define MOVE_BASE_BEHAVIOUR_HPP

#include <vector>
#include <behaviortree_ros2/bt_action_node.hpp>
#include "move_base_skill/action/move_base_skill.hpp"

using namespace BT;

/**
 * A behaviour for interacting with an action of type move_base_skill::action::MoveBaseSkill.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class MoveBaseBehaviour : public RosActionNode<move_base_skill::action::MoveBaseSkill>
{
public:
    MoveBaseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the action goal:
     * - Input ports:
     *     * navigation_goal (move_base_skill::action::MoveBaseSkill::Goal)
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