class PlannerNames(object):
    LAMA = 'lama'

def get_planner_class_from_name(planner_name: str):
    if planner_name == PlannerNames.LAMA:
        from task_planner_interface.planner_interfaces.lama_interface import LAMAInterface
        return LAMAInterface