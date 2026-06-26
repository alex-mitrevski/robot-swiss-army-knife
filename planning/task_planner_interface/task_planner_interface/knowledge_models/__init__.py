from task_planner_interface import DomainNames

def get_knowledge_model_classes(domain_name):
    if domain_name == DomainNames.SERVICE_ROBOT_DOMAIN:
        return 'task_planner_interface.knowledge_models.service_robot_models', ('ServiceRobotFluentLibrary', 'ServiceRobotNumericFluentLibrary')
    elif domain_name == DomainNames.HOSPITAL_TRANSPORTATION_DOMAIN:
        return 'task_planner_interface.knowledge_models.hospital_transportation_models', ('HospitalTransportationFluentLibrary', 'HospitalTransportationNumericFluentLibrary')
    else:
        raise ValueError(f'Unknown domain name {domain_name}')