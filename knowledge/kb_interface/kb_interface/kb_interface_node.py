import threading

import rclpy
from rclpy.node import Node

from robot_swiss_knife_msgs.msg import Fluent, KnowledgeItem
from robot_swiss_knife_msgs.srv import GetFluentAssertions, UpdateKB
from kb_interface.knowledge_base_interface import KnowledgeBaseInterface

class KBInterface(Node):
    """A component exposing services for interacting with a knowledge base.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    # name assigned to the object
    name = None

    # knowledge base interface instance
    kb_interface = None

    # ROS2 fluent retrieval server instance
    fluent_retrieval_server = None

    # ROS2 knowledge base update server instance
    update_kb_server = None

    def __init__(self, name='kb_interface'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name

        kb_db_name = self.get_parameter_or('kb_db_name', rclpy.Parameter('kb_db_name', rclpy.Parameter.Type.STRING, 'robot_store')).value
        get_fluent_assertions_srv_name = self.get_parameter_or('get_fluent_assertions_srv_name',
                                                               rclpy.Parameter('get_fluent_assertions_srv_name', rclpy.Parameter.Type.STRING, 'get_fluent_assertions')).value
        update_kb_srv_name = self.get_parameter_or('update_kb_srv_name',
                                                   rclpy.Parameter('update_kb_srv_name', rclpy.Parameter.Type.STRING, 'update_kb')).value

        self.init_kb_interface(kb_db_name=kb_db_name)

        self.get_logger().info(f'[{self.name}] Exposing fluent retrieval service "{get_fluent_assertions_srv_name}"')
        self.fluent_retrieval_server = self.create_service(GetFluentAssertions,
                                                           get_fluent_assertions_srv_name,
                                                           self.get_fluent_assertions_cb)

        self.get_logger().info(f'[{self.name}] Exposing knowledge base update service "{update_kb_srv_name}"')
        self.update_kb_server = self.create_service(UpdateKB,
                                                    update_kb_srv_name,
                                                    self.update_kb_cb)

        self.get_logger().info(f'[{self.name}] Knowledge base interface ready')

    def init_kb_interface(self, kb_db_name: str) -> None:
        """Initialises a knowledge base interface.        

        Keyword arguments:
        kb_db_name: str -- Name of a database that should be used for storing the knowledge base

        """
        self.get_logger().info(f'[{self.name}] Initialising knowledge base interface')
        self.kb_interface = KnowledgeBaseInterface(kb_db_name)
        self.get_logger().info(f'[{self.name}] Knowledge base interface initialised successfully')

    def get_fluent_assertions_cb(self, request: GetFluentAssertions.Request,
                                 response: GetFluentAssertions.Response) -> GetFluentAssertions.Response:
        """Retrieves fluent assertions from the knowledge base. If request.name is not empty,
        returns only the assertions belonging to that fluent; otherwise, returns all assertions.

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.GetFluentAssertions.Request
        response: robot_swiss_army_msgs.srv.GetFluentAssertions.Response

        """
        if request.name:
            self.get_logger().info(f'[{self.name}] Received new fluent assertion retrieval request for fluent {request.name}')
        else:
            self.get_logger().info(f'[{self.name}] Received new fluent assertion retrieval request; retrieving all fluents')

        fluent_assertions = self.kb_interface.get_fluent_assertions()
        for assertion in fluent_assertions:
            if request.name and request.name != assertion.name:
                continue

            f = Fluent()
            f.name = assertion.name
            f.value = assertion.value
            for param in assertion.params:
                knowledge_item = KnowledgeItem()
                knowledge_item.name = param.name
                knowledge_item.value = param.value
                f.params.append(knowledge_item)
            response.fluents.append(f)

        self.get_logger().info(f'[{self.name}] Fluent retrieval complete')
        return response

    def update_kb_cb(self, request: UpdateKB.Request,
                     response: UpdateKB.Response) -> UpdateKB.Response:
        """Updates the knowledge base with the fluents passed in the request, performing
        the operation (insert or remove) specified in the request. If any errors occur
        during the knowledge base update, the "success" field of the response is set to False.

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.UpdateKB.Request
        response: robot_swiss_army_msgs.srv.UpdateKB.Response

        """
        self.get_logger().info(f'[{self.name}] Received a new knowledge base update request (type {request.operation})')

        fluent_tuple_list = []
        for fluent in request.fluents:
            param_tuple_list = []
            for param in fluent.params:
                param_tuple_list.append((param.name, param.value))
            fluent_tuple_list.append((fluent.name, param_tuple_list, fluent.value))

        try:
            self.get_logger().info(f'[{self.name}] Knowledge base update complete')
            if request.operation == UpdateKB.Request.INSERT:
                self.kb_interface.insert_fluents(fluent_tuple_list)
            elif request.operation == UpdateKB.Request.REMOVE:
                self.kb_interface.remove_fluents(fluent_tuple_list)
            else:
                raise ValueError(f'Received unknown knowledge update operation: {request.operation}')
            response.success = True
        except Exception as exc:
            self.get_logger().error(f'[{self.name}] Error during knowledge base update: {str(exc)}')
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    kb_interface = KBInterface()
    rate = kb_interface.create_rate(5, kb_interface.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(kb_interface,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except:
        pass

    print('Destroying node')
    kb_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()