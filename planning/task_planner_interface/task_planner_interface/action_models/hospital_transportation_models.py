import uuid

from task_planner_interface.action_models.action_model_base import ActionModelBase, Action

class HospitalTransportationActionModels(ActionModelBase):
    def __init__(self):
        super(HospitalTransportationActionModels, self).__init__()

    @staticmethod
    def get_action_model(action_name: str, action_params: list) -> Action:
        action = Action()
        action.id = str(uuid.uuid4())
        action.type = action_name
        action = getattr(HospitalTransportationActionModels, action_name)(action, action_params)
        return action

    @staticmethod
    def GOTO(action: Action, params: list) -> Action:
        action.params = {'destination': params[2]}
        return action

    @staticmethod
    def DOCK(action: Action, params: list) -> Action:
        action.params = {'load': params[1]}
        return action

    @staticmethod
    def UNDOCK(action: Action, params: list) -> Action:
        return action

    @staticmethod
    def REQUEST_ELEVATOR(action: Action, params: list) -> Action:
        return action

    @staticmethod
    def WAIT_FOR_ELEVATOR(action: Action, params: list) -> Action:
        return action

    @staticmethod
    def ENTER_ELEVATOR(action: Action, params: list) -> Action:
        return action

    @staticmethod
    def RIDE_ELEVATOR(action: Action, params: list) -> Action:
        return action

    @staticmethod
    def EXIT_ELEVATOR(action: Action, params: list) -> Action:
        action.params = {'destination': params[1]}
        return action
