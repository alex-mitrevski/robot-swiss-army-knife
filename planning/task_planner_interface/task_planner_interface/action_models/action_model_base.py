class Action(object):
    def __init__(self):
        self.id = ''
        self.type = ''
        self.params = {}

    def to_dict(self) -> dict:
        '''Converts the object to a dictionary with two keys: "name" and "value".
        '''
        dict_params = {}
        dict_params['type'] = self.type
        dict_params['params'] = self.params
        return dict_params

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return f"Action({str(self.to_dict())})"

class ActionModelBase(object):
    pass