from kb_interface.knowledge_models.knowledge_models_base import PDDLKnowledgeUtils, PDDLFluentLibrary, PDDLNumericFluentLibrary

class HospitalTransportationFluentLibrary(PDDLFluentLibrary):
    def __init__(self):
        super(HospitalTransportationFluentLibrary, self).__init__()

    @staticmethod
    def get_assertion_param_list(fluent_name: str, fluent_params: list,
                                 fluent_value: str, obj_types: dict) -> tuple[list, dict]:
        ordered_param_list, obj_types = getattr(HospitalTransportationFluentLibrary, fluent_name)(fluent_params,
                                                                                                  obj_types,
                                                                                                  fluent_value)
        return ordered_param_list, obj_types

    @staticmethod
    def empty_gripper(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def holding(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: 'bot', 1: 'load'}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def requested(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot'), 1: ('elevator', 'elevator')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def arrived(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def elevator_at(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator'), 1: ('loc', 'location')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)



    @staticmethod
    def robot_at(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'location', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def robot_in(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'elevator', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def load_at(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('load', 'load')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'location', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def load_in(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('load', 'load')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'elevator', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def robot_floor(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'floor', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def load_floor(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('load', 'load')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'floor', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def location_floor(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('loc', 'location')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'floor', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def elevator_floor(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'floor', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def destination_floor(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'floor', updated_obj_types)
        return ordered_param_list, updated_obj_types


class HospitalTransportationNumericFluentLibrary(PDDLNumericFluentLibrary):
    def __init__(self):
        super(HospitalTransportationNumericFluentLibrary, self).__init__()

    @staticmethod
    def get_assertion_param_list(fluent_name: str, fluent_params: list, obj_types: dict) -> tuple[list, dict]:
        ordered_param_list, obj_types = getattr(HospitalTransportationNumericFluentLibrary, fluent_name)(fluent_params, obj_types)
        return ordered_param_list, obj_types

    @staticmethod
    def robot_floor(params: list, obj_types: dict) -> tuple[list, dict]:
        param_order = {0: ('bot', 'robot')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def load_floor(params: list, obj_types: dict) -> tuple[list, dict]:
        param_order = {0: ('load', 'load')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def location_floor(params: list, obj_types: dict) -> tuple[list, dict]:
        param_order = {0: ('loc', 'location')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def elevator_floor(params: list, obj_types: dict) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def destination_floor(params: list, obj_types: dict) -> tuple[list, dict]:
        param_order = {0: ('elevator', 'elevator')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)