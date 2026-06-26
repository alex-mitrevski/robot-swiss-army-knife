import yaml
from kb_interface.knowledge_base_interface import KnowledgeItemType
from task_planner_interface.planner_interfaces.lama_interface import LAMAInterface

if __name__ == '__main__':
    with open('../config/planner_config.yaml', 'r') as config_file:
        planner_config = yaml.safe_load(config_file)

    domain_file = planner_config['domain_file']
    planner_cmd = planner_config['planner_cmd']
    plan_file_path = planner_config['plan_file_path']

    state_fluents = [('robotName', [('Robot', 'MyRobot')], 'true'),
                     ('robotAt', [('Robot', 'MyRobot')], 'Kitchen'),
                     ('planeAt', [('Plane', 'CoffeeTable')], 'LivingRoom'),
                     ('planeAt', [('Plane', 'DiningTable')], 'DiningRoom'),
                     ('objectOnPlane', [('Object', 'WaterBottle')], 'CoffeeTable'),
                     ('objectOnPlane', [('Object', 'CoffeeMug')], 'CoffeeTable'),
                     ('objectOnPlane', [('Object', 'MySoupPlate')], 'DiningTable'),
                     ('unexplored', [('Plane', 'CoffeeTable')], 'true'),
                     ('emptyGripper', [('Robot', 'MyRobot')], 'true')]

    planner_interface = LAMAInterface('test_kb', domain_file, planner_cmd, plan_file_path, debug=True)
    planner_interface.kb_interface.insert_fluents(state_fluents, KnowledgeItemType.FACT)

    task_goals = [('objectOnPlane', [('Object', 'WaterBottle')], 'DiningTable'),
                    ('emptyGripper', [('Robot', 'MyRobot')], 'true')]
    planner_interface.kb_interface.insert_fluents(task_goals, KnowledgeItemType.GOAL)

    plan_found, plan = planner_interface.plan()
    print(f'Found plan? {plan_found}')
    for action in plan:
        print(f'{action}')

    planner_interface.kb_interface.remove_fluents(state_fluents, KnowledgeItemType.FACT)
    planner_interface.kb_interface.remove_fluents(task_goals, KnowledgeItemType.GOAL)
    planner_interface.kb_interface.db_client.drop_database('test_kb')