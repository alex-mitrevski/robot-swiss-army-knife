import uuid

from task_planner_interface.action_models.action_model_base import ActionModelBase, Action

class ServiceRobotActionModels(ActionModelBase):
    def __init__(self):
        super(ServiceRobotActionModels, self).__init__()

    @staticmethod
    def get_action_model(action_name: str, action_params: list) -> Action:
        action = Action()
        action.id = str(uuid.uuid4())
        action.type = action_name
        action = getattr(ServiceRobotActionModels, action_name)(action, action_params)
        return action

    @staticmethod
    def MOVEBASE(action: Action, params: list) -> Action:
        action.params = {'destination': params[1]}
        return action

    @staticmethod
    def OPEN(action: Action, params: list) -> Action:
        action.params = {'door': params[0]}
        return action

    @staticmethod
    def PERCEIVEPLANE(action: Action, params: list) -> Action:
        action.params = {'plane': params[0]}
        return action

    @staticmethod
    def PICKFROMPLANE(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'plane': params[1], 'context': params[-1]}
        return action

    @staticmethod
    def PICKFROMCONTAINER(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'container': params[1], 'context': params[-1]}
        return action

    @staticmethod
    def PLACEONPLANE(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'plane': params[1], 'context': params[-1]}
        return action

    @staticmethod
    def PLACEINCONTAINER(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'container': params[1], 'context': params[-1]}
        return action

    @staticmethod
    def THROW(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'target': params[1]}
        return action

    @staticmethod
    def HANDOVER(action: Action, params: list) -> Action:
        action.params = {'object': params[0], 'person': params[2]}
        return action