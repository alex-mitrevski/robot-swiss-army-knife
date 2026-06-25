# kb_interface

A Python-based package that exposes an interface (a Python package and a ROS 2 interface on top of that) for interacting with a logic-based knowledge base.

The Python interface is reused from [this repository](https://github.com/alex-mitrevski/task-planner/tree/master/task_planner). Thus, the knowledge base is stored in a MongoDB database, so a MongoDB instance should be active for using the component (I interact with MongoDB over Docker, using the [official Docker image](https://hub.docker.com/_/mongo)).

## Dependencies

* [robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs)

## Usage instructions

There are two ways to use the knowledge base:
* As an ordinary Python component
* Over ROS 2 (this is particularly useful for C++-based components)

### Interaction over Python

Python components can simply interact with the knowledge base through an instance of the `KnowledgeBaseInterface` class from the `kb_interface.knowledge_base_interface` package. Examples of interacting with the knowledge base using the Python interface can be found in [`kb_python_interaction_test.py`](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/knowledge/kb_interface/test/kb_python_interaction_test.py) in the `test` directory.

### Interaction over ROS

The component also includes a node that exposes knowledge base interaction services. After building this package, the node can be started as follows:
```
ros2 launch kb_interface kb_interface.launch.py
```

The component exposes two services:
* A service for retrieving fluent assertions. The interface used by this service is of type [robot_swiss_knife_msgs/srv/GetFluentAssertions](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/GetFluentAssertions.srv). If the name in the request is left empty, all assertions will be retrieved from the knowledge base.
* A service for updating the knowledge base (inserting or removing assertions). The interface used by this service is of type [robot_swiss_knife_msgs/srv/UpdateKB](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/UpdateKB.srv).

The names of the services (`get_fluent_assertions` and `update_kb` by default) can be specified in the launch file (both of these exist in the `/kb` namespace by default). The name of the database used for knowledge base storage can also be specified through the launch file (default `robot_store`).

An example of a Python-based ROS 2 node for interacting with the knowledge base can be found in [`kb_ros_interaction_test.py`](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/knowledge/kb_interface/test/kb_ros_interaction_test.py) in the `test` directory.

## Additional notes

* The component uses MongoDB for storing the knowledge base rather than keeping it in memory (as is done in systems such as [PlanSys2](https://plansys2.github.io/index.html)) so that knowledge can persist even if the component itself or a robot crashes.
* The component includes what I call "knowledge models", though these are only relevant during task planning (as they create a direct mapping with predicates in specific planning domains). The knowledge models included here are for [a service robot domain](https://github.com/alex-mitrevski/task-planner/blob/master/config/task_domains/service_robot_domain.pddl) and [a hospital transportation domain](https://github.com/alex-mitrevski/task-planner/blob/master/config/task_domains/hospital_transportation.pddl). I will expand on these at a later point, once I integrate a task planning interface in this repository.