import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robot_swiss_knife_msgs.msg import Fluent, KnowledgeItem
from robot_swiss_knife_msgs.srv import GetFluentAssertions, UpdateKB
from robot_swiss_knife_msgs.action import GetTaskPlan


class TaskPlannerInterfaceTest(Node):
    def __init__(self):
        super().__init__('task_planner_interface_test')
        self.knowledge_retrieval_client = self.create_client(GetFluentAssertions, '/planner/get_fluent_assertions')
        self.goal_retrieval_client = self.create_client(GetFluentAssertions, '/planner/get_goals')
        self.update_kb_client = self.create_client(UpdateKB, '/planner/update_kb')
        self.update_goals_client = self.create_client(UpdateKB, '/planner/update_goals')
        self.plan_client = ActionClient(self, GetTaskPlan, '/planner/plan')

        while not self.knowledge_retrieval_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/planner/get_fluent_assertions service not available, waiting...')

        while not self.goal_retrieval_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/planner/get_goals service not available, waiting...')

        while not self.update_kb_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/planner/update_kb service not available, waiting...')

        while not self.update_goals_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/planner/update_goals service not available, waiting...')

        self.plan_result = None

    def get_fluent_assertions(self, name=''):
        retrieval_request = GetFluentAssertions.Request()
        retrieval_request.name = name
        self.future = self.knowledge_retrieval_client.call_async(retrieval_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def get_goals(self):
        retrieval_request = GetFluentAssertions.Request()
        self.future = self.goal_retrieval_client.call_async(retrieval_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def insert_fluents(self, fluent_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.INSERT
        update_kb_request.fluents = self.get_fluents(fluent_tuple_list)
        self.future = self.update_kb_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def remove_fluents(self, fluent_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.REMOVE
        update_kb_request.fluents = self.get_fluents(fluent_tuple_list)
        self.future = self.update_kb_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def insert_goals(self, goal_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.INSERT
        update_kb_request.fluents = self.get_fluents(goal_tuple_list)
        self.future = self.update_goals_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def remove_goals(self, goal_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.REMOVE
        update_kb_request.fluents = self.get_fluents(goal_tuple_list)
        self.future = self.update_goals_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def call_planner(self):
        self.plan_client.wait_for_server()
        self.plan_future = self.plan_client.send_goal_async(GetTaskPlan.Goal())
        self.plan_future.add_done_callback(self.plan_response_cb)

    def plan_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.plan_result_future = goal_handle.get_result_async()
        self.plan_result_future.add_done_callback(self.get_plan_result_cb)

    def get_plan_result_cb(self, future):
        result = future.result().result
        self.plan_result = result

    def get_fluents(self, fluent_tuple_list):
        fluents = []
        for fluent_name, fluent_params, fluent_value in fluent_tuple_list:
            f = Fluent()
            f.name = fluent_name
            f.value = fluent_value
            for param_name, param_value in fluent_params:
                item = KnowledgeItem()
                item.name = param_name
                item.value = param_value
                f.params.append(item)
            fluents.append(f)
        return fluents

    def print_assertions(self, fluent_assertions):
        for fluent in fluent_assertions:
            fluent_string = f'{fluent.name} -- '
            param_string = ''
            for param in fluent.params:
                param_string += f'({param.name}, {param.value}) '
            fluent_string += f'[ {param_string}] -- {fluent.value}'
            self.get_logger().info(fluent_string)
        print()

    def print_plan(self, plan):
        for action in plan:
            action_string = f'{action.name} -- '
            param_string = ''
            for param in action.params:
                param_string += f'({param.name}, {param.value}) '
            action_string += f'[ {param_string}]'
            self.get_logger().info(action_string)
        print()

def main(args=None):
    rclpy.init(args=args)
    task_planner_interface_test = TaskPlannerInterfaceTest()

    state_fluents = [('robotName', [('Robot', 'MyRobot')], 'true'),
                    ('robotAt', [('Robot', 'MyRobot')], 'Kitchen'),
                    ('planeAt', [('Plane', 'CoffeeTable')], 'LivingRoom'),
                    ('planeAt', [('Plane', 'DiningTable')], 'DiningRoom'),
                    ('objectOnPlane', [('Object', 'WaterBottle')], 'CoffeeTable'),
                    ('objectOnPlane', [('Object', 'CoffeeMug')], 'CoffeeTable'),
                    ('objectOnPlane', [('Object', 'MySoupPlate')], 'DiningTable'),
                    ('unexplored', [('Plane', 'CoffeeTable')], 'true'),
                    ('emptyGripper', [('Robot', 'MyRobot')], 'true')]

    task_goals = [('objectOnPlane', [('Object', 'WaterBottle')], 'DiningTable'),
                    ('emptyGripper', [('Robot', 'MyRobot')], 'true')]

    try:
        ####################################
        # Fact retrieval and insertion tests
        ####################################
        task_planner_interface_test.get_logger().info('Retrieving assertions before insertion')
        response = task_planner_interface_test.get_fluent_assertions()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == 0

        task_planner_interface_test.get_logger().info('Inserting assertions')
        task_planner_interface_test.insert_fluents(state_fluents)

        task_planner_interface_test.get_logger().info('Retrieving assertions after insertion')
        response = task_planner_interface_test.get_fluent_assertions()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == len(state_fluents)

        task_planner_interface_test.get_logger().info('Retrieving assertions of type robotAt')
        response = task_planner_interface_test.get_fluent_assertions('robotAt')
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == 1

        ####################################
        # Goal retrieval and insertion tests
        ####################################
        task_planner_interface_test.get_logger().info('Retrieving goals')
        response = task_planner_interface_test.get_goals()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == 0

        task_planner_interface_test.get_logger().info('Inserting goals')
        task_planner_interface_test.insert_goals(task_goals)

        task_planner_interface_test.get_logger().info('Retrieving goals after insertion')
        response = task_planner_interface_test.get_goals()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == len(task_goals)

        ###############
        # Planning test
        ###############
        task_planner_interface_test.get_logger().info('Planning')
        task_planner_interface_test.call_planner()
        while task_planner_interface_test.plan_result is None:
            rclpy.spin_once(task_planner_interface_test)
        task_planner_interface_test.get_logger().info(f'Plan found? {task_planner_interface_test.plan_result.success}')
        task_planner_interface_test.print_plan(task_planner_interface_test.plan_result.plan)

        ###################
        # Goal removal test
        ###################
        task_planner_interface_test.get_logger().info('Removing goals')
        task_planner_interface_test.remove_goals(task_goals)

        task_planner_interface_test.get_logger().info('Retrieving goals after removal')
        response = task_planner_interface_test.get_goals()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == 0

        ###################
        # Fact removal test
        ###################
        task_planner_interface_test.get_logger().info('Removing assertions')
        task_planner_interface_test.remove_fluents(state_fluents)

        task_planner_interface_test.get_logger().info('Retrieving assertions after removal')
        response = task_planner_interface_test.get_fluent_assertions()
        task_planner_interface_test.print_assertions(response.fluents)
        assert len(response.fluents) == 0
    except Exception as exc:
        print(exc)

    print('Destroying node')
    task_planner_interface_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()