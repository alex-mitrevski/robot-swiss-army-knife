class DomainNames(object):
    SERVICE_ROBOT_DOMAIN = 'service-robot-domain'
    HOSPITAL_TRANSPORTATION_DOMAIN = 'hospital-transportation'

def get_knowledge_model_classes(domain_name):
    if domain_name == DomainNames.SERVICE_ROBOT_DOMAIN:
        return 'kb_interface.knowledge_models.service_robot_models', ('ServiceRobotFluentLibrary', 'ServiceRobotNumericFluentLibrary')
    elif domain_name == DomainNames.HOSPITAL_TRANSPORTATION_DOMAIN:
        return 'kb_interface.knowledge_models.hospital_transportation_models', ('HospitalTransportationFluentLibrary', 'HospitalTransportationNumericFluentLibrary')
    else:
        raise ValueError(f'Unknown domain name {domain_name}')