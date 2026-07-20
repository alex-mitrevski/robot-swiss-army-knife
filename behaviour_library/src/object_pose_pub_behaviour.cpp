#include "behaviour_library/object_pose_pub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

ObjectPosePubBehaviour::ObjectPosePubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{}

PortsList ObjectPosePubBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<robot_swiss_knife_msgs::msg::Object>("object")
    });
}

bool ObjectPosePubBehaviour::setMessage(geometry_msgs::msg::PoseStamped& pose_stamped_msg)
{
    robot_swiss_knife_msgs::msg::Object obj;
    this->getInput("object", obj);
    pose_stamped_msg = obj.pose;
    return true;
}

CreateRosNodePlugin(ObjectPosePubBehaviour, "ObjectPosePubBehaviour");