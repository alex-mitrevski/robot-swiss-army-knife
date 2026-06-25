import pymongo as pm
from bson.objectid import ObjectId


class AssertionTypes(object):
    FLUENT = 'fluent'


class PredicateParams(object):
    """An object representing a predicate parameter (variable name and ground value).
    """
    def __init__(self):
        self.name = ''
        self.value = ''

    def __eq__(self, other) -> bool:
        """Returns True if both the names and the values are the same.
        """
        return self.name == other.name and self.value == other.value

    def __ne__(self, other) -> bool:
        """Returns True if either the name or the value differ.
        """
        return self.name != other.name or self.value != other.value

    def to_dict(self) -> dict:
        """Converts the object to a dictionary with two keys: "name" and "value".
        """
        dict_params = {}
        dict_params['name'] = self.name
        dict_params['value'] = self.value
        return dict_params

    def to_tuple(self) -> tuple[str, str]:
        """Convert object to tuple(str, str)
        """
        return (self.name, self.value)

    @staticmethod
    def from_tuple(tuple_params: tuple[str, str]):
        """Returns a PredicateParams object created from the input tuple.

        Keyword arguments:
        @param tuple_params -- a tuple with two entries - "name" and "value"

        """
        params = PredicateParams()
        params.name, params.value = tuple_params
        return params

    @staticmethod
    def from_dict(dict_params: dict):
        """Returns a PredicateParams object created from the input dictionary.

        Keyword arguments:
        @param dict_params -- a dictionary with two keys - "name" and "value"

        """
        params = PredicateParams()
        params.name = dict_params['name']
        params.value = dict_params['value']
        return params

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return "PredicateParams(" + str(self.to_dict()) + ")"

class Fluent(object):
    """An object representing a fluent (fluent name, list of ground values, and fluent value).
    """
    def __init__(self):
        self.name = ''
        self.params = []
        self.value = None

    def __eq__(self, other) -> bool:
        """Returns True if the names, values, and all parameters are the same.
        """
        equal = False
        if self.name == other.name and self.value == other.value:
            equal = True
            for param in self.params:
                if param not in other.params:
                    equal = False
                    break
        return equal

    def to_dict(self) -> dict:
        """Converts the object to a dictionary with three keys - "name", "params", and "value".
        The value of "params" is a list of PredicateParams dictionaries.
        """
        dict_fluent = {}
        dict_fluent['name'] = self.name
        dict_fluent['type'] = AssertionTypes.FLUENT
        dict_fluent['value'] = self.value
        dict_fluent['params'] = []
        for param_data in self.params:
            dict_params = param_data.to_dict()
            dict_fluent['params'].append(dict_params)
        return dict_fluent

    def to_tuple(self) -> tuple[str, list, str]:
        """Convert the object to a tuple for with three elements:
        name -- string
        params -- list of tuple(str, str)
        value -- int or string
        """
        return (self.name, [param.to_tuple() for param in self.params], self.value)

    @staticmethod
    def from_tuple(tuple_fluent: tuple):
        """Returns a Fluent object created from the input tuple.

        Keyword arguments:
        @param tuple_fluent -- a tuple with three entries, the first representing
                               the name of the predicate, the second a list of
                               ("name", "value") pairs for the predicate parameters,
                               and the third the fluent value

        """
        fluent = Fluent()
        fluent.name, tuple_data, fluent.value = tuple_fluent
        for tuple_params in tuple_data:
            params = PredicateParams.from_tuple(tuple_params)
            fluent.params.append(params)
        return fluent

    @staticmethod
    def from_dict(dict_fluent: dict):
        """Returns a Fluent object created from the input dictionary.

        Keyword arguments:
        @param dict_fluent-- a dictionary with three keys - "name", "params", and "value",
                             where "params" is a list of PredicateParams dictionaries

        """
        fluent = Fluent()
        fluent.name = dict_fluent['name']
        fluent.value = dict_fluent['value']
        dict_data = dict_fluent['params']
        for dict_params in dict_data:
            params = PredicateParams.from_dict(dict_params)
            fluent.params.append(params)
        return fluent

    def __str__(self) -> str:
        string = "Fluent(\n"
        string += '\t' + 'name:' + str(self.name) + '\n'
        string += '\t' + 'value:' + str(self.value) + '\n'
        string += '\t' + 'type:' + str(AssertionTypes.FLUENT) + '\n'
        string += '\t' + 'params:[' + '\n'
        for param in self.params:
            string += '\t\t' + str(param) + '\n'
        string += '\t' + ']' + '\n'
        string += ")"
        return string

    def __repr__(self) -> str:
        return "Fluent(" + str(self.to_dict()) + ")"

class KnowledgeBaseInterface(object):
    """Defines an interface for interacting with a robot knowledge base.

    Constructor arguments:
    @param kb_database_name -- name of a database in which the knowledge base will be stored

    """
    def __init__(self, kb_database_name='robot_store'):
        self.db_client = pm.MongoClient()
        self.kb_database_name = kb_database_name
        self.kb_collection_name = 'knowledge_base'

    def get_fluent_names(self) -> list:
        """Returns a list of all stored fluent names in the knowledge base.
        """
        collection = self.get_kb_collection(self.kb_collection_name)
        fluent_cursor = collection.find({'type': AssertionTypes.FLUENT})
        names = list({f['name'] for f in fluent_cursor})
        return names

    def get_fluent_assertions(self, fluent_name: str=None) -> list:
        """Returns a list of Fluent objects representing all assertions
        of the given fluent in the knowledge base. If "fluent_name" is None,
        returns all fluent assertions in the knowledge base.

        Keyword arguments:
        @param fluent_name: str -- name of a fluent in the knowledge base
                                   (default None, in which case all
                                   assertions are retrieved)
        """
        instances = []
        collection = self.get_kb_collection(self.kb_collection_name)
        if fluent_name:
            fluent_instance_count = collection.count_documents({'name': fluent_name})
            if fluent_instance_count == 0:
                return []

            fluent_instance_cursor = collection.find({'name': fluent_name})
            instances = [Fluent.from_dict(p) for p in fluent_instance_cursor]
        else:
            assertion_cursor = collection.find({'type': AssertionTypes.FLUENT})
            instances = [Fluent.from_dict(p) for p in assertion_cursor]
        return instances

    def get_fluent_value(self, fluent: tuple[str, list]) -> list:
        """Returns the value of the given fluent in the knowledge base.
        Returns None if an assertion for the fluent is not found.

        Keyword arguments:
        @param fluent: tuple[str, list] -- a tuple representing a fluent, where
                                           the first entry is the fluent name and
                                           the second entry is a list of fluent parameters

        """
        fluent_value = None

        # we add a dummy fluent value so that we can create a fluent dictionary
        fluent_full = (fluent[0], fluent[1], -1)
        fluent_dict = Fluent.from_tuple(fluent_full).to_dict()

        collection = self.get_kb_collection(self.kb_collection_name)
        fluent_cursor = collection.find({'name': fluent_dict['name']})
        fluent_assertion = None
        for f in fluent_cursor:
            found = True
            for param in f['params']:
                if param not in fluent_dict['params']:
                    found = False
                    break
            if found:
                fluent_assertion = f
                break

        if fluent_assertion:
            fluent_value = fluent_assertion['value']
        else:
            print('Fluent %s not found', fluent_dict['name'])
        return fluent_value

    def insert_fluents(self, fluent_list: list) -> bool:
        """Inserts a list of fluents into the knowledge base.

        Keyword arguments:
        @param fluents_to_add: list -- fluents to add to the knowledge base. The entries are
                                       tuples with three entries of the form
                                       (name, [parameters], value), namely the first entry
                                       represents the name of the fluent, the second
                                       a list of ("name", "value") pairs for the
                                       fluent parameters, and the third the fluent value

        """
        try:
            self.__insert_fluents(fluent_list, self.kb_collection_name)
            return True
        except Exception as exc:
            print('[insert_fluents] Fluents could not be inserted:', exc_info=True)
            return False

    def remove_fluents(self, fluent_list: list) -> bool:
        """Removes a list of fluents from the knowledge base.

        Keyword arguments:
        @param fluents_to_remove: list -- fluents to remove from the knowledge base. The entries
                                          are tuples with two entries of the form
                                          (name, [parameters], value), namely the first entry
                                          represents the name of the fluent and the second
                                          a list of ("name", "value") pairs for the
                                          fluent parameters

        """
        try:
            self.__remove_fluents(fluent_list, self.kb_collection_name)
            return True
        except Exception as exc:
            print('[remove_fluents] Fluents could not be removed: ', exc_info=True)
            return False

    def update_fluent(self, fluent: tuple[str, list, int]) -> bool:
        """Updates the given fluent in the knowledge base. The fluent
        will be inserted if it does not already exist. Returns True if
        the update is successful; returns False in case of any exceptions.

        Keyword arguments:
        @param fluent: tuple[str, list, int] -- a tuple with three entries, the first representing
                                                the name of the predicate, the second a list of
                                                ("name", "value") pairs for the fluent parameters,
                                                and the third the fluent value

        """
        fluent_name = fluent[0]
        try:
            fluent_obj = Fluent.from_tuple(fluent)
            fluent_dict = fluent_obj.to_dict()

            collection = self.get_kb_collection(self.kb_collection_name)
            collection.replace_one({'name': fluent_name,
                                    'type': AssertionTypes.FLUENT},
                                    fluent_dict, upsert=True)
            return True
        except Exception as exc:
            print('[update_fluent] Fluent {0} could not be updated'.format(fluent_name), exc_info=True)
            return False

    def get_kb_collection(self, collection_name: str) -> pm.collection.Collection:
        """Returns a pymongo collection with the given name.

        Keyword arguments:
        @param collection_name: str -- name of a MongoDB collection

        """ 
        db = self.db_client[self.kb_database_name]
        collection = db[collection_name]
        return collection

    def item_exists(self, item: dict, item_type: str, collection_name: str) -> ObjectId:
        """Returns True if the given fluent exists in the knowledge base.

        Keyword arguments:
        @param item: dict -- a dictionary representation of a Fluent object
        @param item_type: str -- an AssertionTypes string indicating whether
                                 the item is a predicate or a fluent

        """
        collection = self.get_kb_collection(collection_name)
        item_cursor = collection.find({'name': item['name'], 'type': item_type})
        exists = False
        object_id = None
        for kb_item in item_cursor:
            exists = True
            for param in kb_item['params']:
                if param not in item['params']:
                    exists = False
                    break
            if exists:
                object_id = kb_item['_id']
                break
        return object_id

    def __insert_fluents(self, fluent_list: list, collection_name: str) -> bool:
        """Inserts a list of fluents into the given collection.
        If a fluent already exists, its value is updated.

        Keyword arguments:
        @param fluent_list: list -- tuple representations of Fluent objects
        @param collection_name: pm.collection.Collection -- a MongoDB collection

        """
        for fluent_tuple in fluent_list:
            fluent = Fluent.from_tuple(fluent_tuple)
            fluent_dict = fluent.to_dict()
            collection = self.get_kb_collection(collection_name)
            if not self.item_exists(fluent_dict, AssertionTypes.FLUENT, collection_name):
                collection.insert_one(fluent_dict)
            else:
                print(f'Fluent {fluent.name} already exists; updating the value')
                collection.replace_one({'name': fluent_dict['name'],
                                        'params': fluent_dict['params']},
                                       fluent_dict)

    def __remove_fluents(self, fluent_list: list, collection_name: str) -> bool:
        """Removes a list of fluents from the given collection.

        Keyword arguments:
        @param fluent_list: list -- tuple representations of Fluent objects
        @param collection_name: pm.collection.Collection -- a MongoDB collection

        """
        collection = self.get_kb_collection(collection_name)
        for fluent_tuple in fluent_list:
            fluent = Fluent.from_tuple(fluent_tuple)
            fluent_dict = fluent.to_dict()
            object_id = self.item_exists(fluent_dict, AssertionTypes.FLUENT, collection_name)
            if object_id:
                collection.delete_one({'_id': object_id})
            else:
                print(f'Fluent {fluent.name} does not exist; nothing to remove')
