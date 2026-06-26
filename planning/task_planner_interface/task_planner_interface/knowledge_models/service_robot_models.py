from kb_interface.knowledge_models.knowledge_models_base import PDDLKnowledgeUtils, PDDLFluentLibrary, PDDLNumericFluentLibrary

class ServiceRobotFluentLibrary(PDDLFluentLibrary):
    def __init__(self):
        super(ServiceRobotFluentLibrary, self).__init__()

    @staticmethod
    def get_assertion_param_list(fluent_name: str, fluent_params: list,
                                 fluent_value: str, obj_types: dict) -> tuple[list, dict]:
        ordered_param_list, obj_types = getattr(ServiceRobotFluentLibrary, fluent_name)(fluent_params,
                                                                                        obj_types,
                                                                                        fluent_value)
        return ordered_param_list, obj_types

    @staticmethod
    def robotName(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def emptyGripper(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def doorOpen(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Door', 'Door')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def unexplored(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def explored(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def unknown(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def known(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)


    @staticmethod
    def objectCategory(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object0', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def robotAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def doorAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Door', 'Door')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def objectAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def furnitureAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Furniture', 'Furniture')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def planeAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def personAt(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Waypoint', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def belongsTo(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def objectOnPlane(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Plane', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def objectOnObject(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object0', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def objectInObject(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object0', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def objectInFurniture(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Furniture', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def robotHoldingObject(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def personHoldingObject(params: list, obj_types: dict, fluent_value: str) -> tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        ordered_param_list, updated_obj_types = PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
        updated_obj_types = PDDLKnowledgeUtils.assign_fluent_value_to_type(fluent_value, 'Object', updated_obj_types)
        return ordered_param_list, updated_obj_types

class ServiceRobotNumericFluentLibrary(PDDLNumericFluentLibrary):
    def __init__(self):
        super(ServiceRobotNumericFluentLibrary, self).__init__()
