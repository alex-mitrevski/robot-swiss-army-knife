import rclpy
from rclpy.node import Node

from robot_swiss_knife_msgs.msg import Fluent, KnowledgeItem
from robot_swiss_knife_msgs.srv import GetFluentAssertions, UpdateKB


class KBInterfaceTest(Node):
    def __init__(self):
        super().__init__('kb_interface_test')
        self.retrieval_client = self.create_client(GetFluentAssertions, '/kb/get_fluent_assertions')
        self.update_client = self.create_client(UpdateKB, '/kb/update_kb')

        while not self.retrieval_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/kb/get_fluent_assertions service not available, waiting...')

        while not self.retrieval_client.wait_for_service(timeout_sec=1.):
            self.get_logger().info('/kb/update_kb service not available, waiting...')

    def get_fluent_assertions(self, name=''):
        retrieval_request = GetFluentAssertions.Request()
        retrieval_request.name = name
        self.future = self.retrieval_client.call_async(retrieval_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def insert_fluents(self, fluent_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.INSERT
        update_kb_request.fluents = self.get_fluents(fluent_tuple_list)
        self.future = self.update_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def remove_fluents(self, fluent_tuple_list):
        update_kb_request = UpdateKB.Request()
        update_kb_request.operation = UpdateKB.Request.REMOVE
        update_kb_request.fluents = self.get_fluents(fluent_tuple_list)
        self.future = self.update_client.call_async(update_kb_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

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

def main(args=None):
    rclpy.init(args=args)
    kb_interface_test = KBInterfaceTest()

    try:
        kb_interface_test.get_logger().info('Retrieving assertions before insertion')
        response = kb_interface_test.get_fluent_assertions()
        kb_interface_test.print_assertions(response.fluents)

        state_fluents = [('robotName', [('Robot', 'MyRobot')], 'true'),
                         ('robotAt', [('Robot', 'MyRobot')], 'Kitchen')]

        kb_interface_test.get_logger().info('Inserting assertions')
        kb_interface_test.insert_fluents(state_fluents)

        kb_interface_test.get_logger().info('Retrieving assertions after insertion')
        response = kb_interface_test.get_fluent_assertions()
        kb_interface_test.print_assertions(response.fluents)

        kb_interface_test.get_logger().info('Retrieving assertions of type robotAt')
        response = kb_interface_test.get_fluent_assertions('robotAt')
        kb_interface_test.print_assertions(response.fluents)

        kb_interface_test.get_logger().info('Removing assertions')
        kb_interface_test.remove_fluents(state_fluents)

        kb_interface_test.get_logger().info('Retrieving assertions after removal')
        response = kb_interface_test.get_fluent_assertions()
        kb_interface_test.print_assertions(response.fluents)
    except Exception as exc:
        print(exc)

    print('Destroying node')
    kb_interface_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()