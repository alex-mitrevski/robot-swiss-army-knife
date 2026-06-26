class PDDLKnowledgeUtils(object):
    @staticmethod
    def get_ordered_param_list(params: list, param_order: dict, obj_types: dict) -> tuple[list, dict]:
        param_list = []
        param_count = 0
        updated_obj_types = dict(obj_types)
        while param_count < len(params):
            for param in params:
                param_name = param_order[param_count][0]
                param_type = param_order[param_count][1]
                if param.name == param_name:
                    param_list.append(param.value)
                    param_count += 1
                    if param_type not in updated_obj_types:
                        updated_obj_types[param_type] = []

                    if param.value not in updated_obj_types[param_type]:
                        updated_obj_types[param_type].append(param.value)
                    break
        return param_list, updated_obj_types

    @staticmethod
    def assign_fluent_value_to_type(fluent_value: str, type_name: str, obj_types: dict):
        if type_name in obj_types:
            if fluent_value not in obj_types[type_name]:
                obj_types[type_name].append(fluent_value)
        else:
            obj_types[type_name] = [fluent_value]
        return obj_types


class PDDLFluentLibrary(object):
    pass


class PDDLNumericFluentLibrary(object):
    pass