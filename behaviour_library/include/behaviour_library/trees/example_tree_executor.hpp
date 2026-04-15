#ifndef EXAMPLE_TREE_CPP
#define EXAMPLE_TREE_CPP

#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

/**
 * Example tree executor (a stripped down version of the sample executor in BehaviorTree.ROS2):
 * https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/humble/btcpp_ros2_samples/src/sample_bt_executor.cpp
 */
class ExampleTreeServer : public BT::TreeExecutionServer
{
public:
    ExampleTreeServer(const rclcpp::NodeOptions& options);

    virtual void onTreeCreated(BT::Tree& tree) override;
    virtual std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;
private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout;
};

#endif