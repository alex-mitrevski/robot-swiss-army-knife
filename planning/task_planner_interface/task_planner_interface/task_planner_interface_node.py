import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from robot_swiss_knife_msgs.msg import Fluent, KnowledgeItem, Action
from robot_swiss_knife_msgs.srv import GetFluentAssertions, UpdateKB
from robot_swiss_knife_msgs.action import GetTaskPlan
from kb_interface.knowledge_base_interface import KnowledgeItemType
from task_planner_interface.planner_interfaces import PlannerNames, get_planner_class_from_name

class TaskPlannerInterface(Node):
    """A component enabling interaction with a task planner and a knowledge base used by the planner.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    # name assigned to the object
    name = None

    # knowledge base interface instance
    planner_interface = None

    # ROS2 fluent retrieval server instance
    fluent_retrieval_server = None

    # ROS2 goal retrieval server instance
    goal_retrieval_server = None

    # ROS2 knowledge base update server instance
    update_kb_server = None

    # ROS2 goal update server instance
    update_goals_server = None

    # ROS2 planning action instance
    planning_server = None

    def __init__(self, name='task_planner_interface'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.name = name

        planner_name = self.get_parameter_or('planner_name', rclpy.Parameter('planner_name', rclpy.Parameter.Type.STRING, PlannerNames.LAMA)).value
        kb_db_name = self.get_parameter_or('kb_db_name', rclpy.Parameter('kb_db_name', rclpy.Parameter.Type.STRING, 'robot_store')).value
        domain_file_path = self.get_parameter_or('domain_file_path', rclpy.Parameter('domain_file_path', rclpy.Parameter.Type.STRING, '')).value
        planner_cmd = self.get_parameter_or('planner_cmd', rclpy.Parameter('planner_cmd', rclpy.Parameter.Type.STRING, '')).value
        plan_file_path = self.get_parameter_or('plan_file_path', rclpy.Parameter('plan_file_path', rclpy.Parameter.Type.STRING, '.')).value

        get_fluent_assertions_srv_name = self.get_parameter_or('get_fluent_assertions_srv_name',
                                                               rclpy.Parameter('get_fluent_assertions_srv_name', rclpy.Parameter.Type.STRING, 'get_fluent_assertions')).value
        get_goals_srv_name = self.get_parameter_or('get_goals_srv_name',
                                                   rclpy.Parameter('get_goals_srv_name', rclpy.Parameter.Type.STRING, 'get_goals')).value
        update_kb_srv_name = self.get_parameter_or('update_kb_srv_name',
                                                   rclpy.Parameter('update_kb_srv_name', rclpy.Parameter.Type.STRING, 'update_kb')).value
        update_goals_srv_name = self.get_parameter_or('update_goals_srv_name',
                                                      rclpy.Parameter('update_goals_srv_name', rclpy.Parameter.Type.STRING, 'update_goals')).value
        plan_action_name = self.get_parameter_or('plan_action_name',
                                                 rclpy.Parameter('plan_action_name', rclpy.Parameter.Type.STRING, 'plan')).value

        self.get_logger().info('Initialising task planner with the following parameters:')
        self.get_logger().info('--------------------------------------------------------')
        self.get_logger().info(f'planner name: {planner_name}')
        self.get_logger().info(f'knowledge base database name: {kb_db_name}')
        self.get_logger().info(f'domain file: {domain_file_path}')
        self.get_logger().info(f'planner command: {planner_cmd}')
        self.get_logger().info(f'directory where temporary plan files will be stored: {plan_file_path}')
        self.get_logger().info('--------------------------------------------------------')

        self.init_planner_interface(planner_name=planner_name,
                                    kb_db_name=kb_db_name,
                                    domain_file_path=domain_file_path,
                                    planner_cmd=planner_cmd,
                                    plan_file_path=plan_file_path)

        self.get_logger().info(f'[{self.name}] Exposing fluent retrieval service "{get_fluent_assertions_srv_name}"')
        self.fluent_retrieval_server = self.create_service(GetFluentAssertions,
                                                           get_fluent_assertions_srv_name,
                                                           self.get_fluent_assertions_cb)

        self.get_logger().info(f'[{self.name}] Exposing goal retrieval service "{get_goals_srv_name}"')
        self.goal_retrieval_server = self.create_service(GetFluentAssertions,
                                                         get_goals_srv_name,
                                                         self.get_goals_cb)

        self.get_logger().info(f'[{self.name}] Exposing knowledge base update service "{update_kb_srv_name}"')
        self.update_kb_server = self.create_service(UpdateKB,
                                                    update_kb_srv_name,
                                                    self.update_kb_cb)

        self.get_logger().info(f'[{self.name}] Exposing goal update service "{update_goals_srv_name}"')
        self.update_goals_server = self.create_service(UpdateKB,
                                                       update_goals_srv_name,
                                                       self.update_goals_cb)

        self.get_logger().info(f'[{self.name}] Exposing planning action "{plan_action_name}"')
        self.planning_server = ActionServer(self, GetTaskPlan, plan_action_name, self.plan_cb)

        self.get_logger().info(f'[{self.name}] Task planner interface ready')

    def init_planner_interface(self, planner_name: str,
                               kb_db_name: str,
                               domain_file_path: str,
                               planner_cmd: str,
                               plan_file_path: str) -> None:
        """Initialises a planner interface with knowledge base access.

        Keyword arguments:
        planner_name: str -- Name of the task planner to be used
        kb_db_name: str -- Name of a database that should be used for storing the knowledge base

        """
        self.get_logger().info(f'[{self.name}] Initialising planner interface')
        PlannerClass = get_planner_class_from_name(planner_name)
        self.planner_interface = PlannerClass(kb_database_name=kb_db_name,
                                              domain_file=domain_file_path,
                                              planner_cmd=planner_cmd,
                                              plan_file_path=plan_file_path)
        self.get_logger().info(f'[{self.name}] Planner interface initialised successfully')

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

        fluent_assertions = self.planner_interface.kb_interface.get_fluent_assertions()
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

    def get_goals_cb(self, request: GetFluentAssertions.Request,
                     response: GetFluentAssertions.Response) -> GetFluentAssertions.Response:
        """Retrieves all goals from the knowledge base.

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.GetFluentAssertions.Request
        response: robot_swiss_army_msgs.srv.GetFluentAssertions.Response

        """
        goals = self.planner_interface.kb_interface.get_goals()
        for goal in goals:
            f = Fluent()
            f.name = goal.name
            f.value = goal.value
            for param in goal.params:
                knowledge_item = KnowledgeItem()
                knowledge_item.name = param.name
                knowledge_item.value = param.value
                f.params.append(knowledge_item)
            response.fluents.append(f)

        self.get_logger().info(f'[{self.name}] Goal retrieval complete')
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
        response.success = self.update_kb(request.fluents, request.operation, KnowledgeItemType.FACT)
        return response

    def update_goals_cb(self, request: UpdateKB.Request,
                        response: UpdateKB.Response) -> UpdateKB.Response:
        """Updates the goals in the knowledge base with the fluents passed in the request, performing
        the operation (insert or remove) specified in the request. If any errors occur
        during the update, the "success" field of the response is set to False.

        Keyword arguments:
        request: robot_swiss_army_msgs.srv.UpdateKB.Request
        response: robot_swiss_army_msgs.srv.UpdateKB.Response

        """
        self.get_logger().info(f'[{self.name}] Received a new goal update request (type {request.operation})')
        response.success = self.update_kb(request.fluents, request.operation, KnowledgeItemType.GOAL)
        return response

    def plan_cb(self, goal_handle: GetTaskPlan.Goal) -> GetTaskPlan.Result:
        """Task planning callback. The result contains the found plan,
        a flag indicating whether there were any errors in the planning,
        and a message that is filled in case of unsuccessful planning or errors.

        Keyword arguments:
        goal_handle: Planning action goal handle

        """
        self.get_logger().info(f'[{self.name}] Received a new planning request')
        result = GetTaskPlan.Result()
        try:
            plan_found, plan = self.planner_interface.plan()
            goal_handle.succeed()
            if plan_found:
                self.get_logger().info(f'[{self.name}] Found the following plan:')
                self.get_logger().info(f'[{self.name}] -----------------------------------------')

                for action in plan:
                    action_msg = Action()
                    action_msg.name = action.type
                    for param_name, param_value in action.params.items():
                        action_param_msg = KnowledgeItem()
                        action_param_msg.name = param_name
                        action_param_msg.value = param_value
                        action_msg.params.append(action_param_msg)
                    self.get_logger().info(f'[{self.name}] {action_msg}')
                    result.plan.append(action_msg)

                self.get_logger().info(f'[{self.name}] -----------------------------------------')
                result.success = True
            else:
                self.get_logger().warn(f'[{self.name}] Plan could not be found')
                result.success = False
                result.message = "Plan could not be found"
        except Exception as exc:
            self.get_logger().error(f'[{self.name}] An error occurred while planning: {str(exc)}')
            result.success = False
            result.message = "Encountered an exception while planning; could not find a plan"
        return result

    def update_kb(self, fluents: list[Fluent], operation: int, knowledge_item_type: str) -> bool:
        """Updates the knowledge base with the given fluents depending on the operation (indicating
        whether to insert or remove them) and the knowledge item type (indicating whether the
        fluents represent facts or goals).

        Keyword arguments:
        fluents: list[Fluent] -- List of fluents to insert or remove
        operation: int -- Indicates the operation to perform with the fluents (i.e. insert or remove them)
        knowledge_item_type: str -- The type of knowledge that the fluents represent (facts or goals)

        """
        fluent_tuple_list = []
        for fluent in fluents:
            param_tuple_list = []
            for param in fluent.params:
                param_tuple_list.append((param.name, param.value))
            fluent_tuple_list.append((fluent.name, param_tuple_list, fluent.value))

        try:
            if operation == UpdateKB.Request.INSERT:
                self.planner_interface.kb_interface.insert_fluents(fluent_tuple_list, knowledge_item_type)
            elif operation == UpdateKB.Request.REMOVE:
                self.planner_interface.kb_interface.remove_fluents(fluent_tuple_list, knowledge_item_type)
            else:
                raise ValueError(f'[{self.name}] Received unknown knowledge update operation: {operation}')

            self.get_logger().info(f'[{self.name}] Knowledge base update complete')
            return True
        except Exception as exc:
            self.get_logger().error(f'[{self.name}] Error during knowledge base update: {str(exc)}')
            return False        

def main(args=None):
    rclpy.init(args=args)
    task_planner_interface = TaskPlannerInterface()
    rate = task_planner_interface.create_rate(5, task_planner_interface.get_clock())

    thread = threading.Thread(target=rclpy.spin, args=(task_planner_interface,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            rate.sleep()
    except:
        pass

    print('Destroying node')
    task_planner_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()