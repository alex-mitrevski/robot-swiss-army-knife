#include "behaviour_library/trees/example_tree_executor.hpp"

ExampleTreeServer::ExampleTreeServer(const rclcpp::NodeOptions& options)
: TreeExecutionServer(options) {}

void ExampleTreeServer::onTreeCreated(BT::Tree& tree)
{
    this->logger_cout = std::make_shared<BT::StdCoutLogger>(tree);
}

std::optional<std::string> ExampleTreeServer::onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)
{
    this->logger_cout.reset();
    return std::nullopt;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto tree_execution_server = std::make_shared<ExampleTreeServer>(options);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
    exec.add_node(tree_execution_server->node());
    exec.spin();
    exec.remove_node(tree_execution_server->node());

    rclcpp::shutdown();
}