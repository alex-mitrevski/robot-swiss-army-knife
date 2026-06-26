from task_planner_interface import DomainNames

def get_action_model_class(domain_name):
    if domain_name == DomainNames.SERVICE_ROBOT_DOMAIN:
        return 'task_planner_interface.action_models.service_robot_models', 'ServiceRobotActionModels'
    elif domain_name == DomainNames.HOSPITAL_TRANSPORTATION_DOMAIN:
        return 'task_planner_interface.action_models.hospital_transportation_models', 'HospitalTransportationActionModels'
    else:
        raise ValueError(f'Unknown domain name {domain_name}')