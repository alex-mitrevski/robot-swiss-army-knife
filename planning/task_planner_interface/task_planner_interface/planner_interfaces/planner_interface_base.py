from abc import abstractmethod
from importlib import import_module

from kb_interface.knowledge_base_interface import KnowledgeBaseInterface
from task_planner_interface.knowledge_models import get_knowledge_model_classes
from task_planner_interface.action_models import get_action_model_class

class TaskPlannerInterfaceBase(object):
    def __init__(self, kb_database_name, domain_file, planner_cmd, plan_file_path, debug=False):
        self.kb_interface = KnowledgeBaseInterface(kb_database_name)

        self.domain_file = domain_file
        self.domain_name = self.__get_domain_name(self.domain_file)
        self.planner_cmd = planner_cmd.replace('DOMAIN', self.domain_file)
        self.plan_file_path = plan_file_path

        knowledge_module, (fluent_lib_name, numeric_fluent_lib_name) = get_knowledge_model_classes(self.domain_name)
        self.fluent_lib = getattr(import_module(knowledge_module), fluent_lib_name)
        self.numeric_fluent_lib = getattr(import_module(knowledge_module), numeric_fluent_lib_name)

        action_module, action_lib_name = get_action_model_class(self.domain_name)
        self.action_model_lib = getattr(import_module(action_module), action_lib_name)

        self.debug = debug

    @abstractmethod
    def plan(self):
        pass

    @abstractmethod
    def generate_problem_file(self, fluent_assertions: list, task_goals: list) -> str:
        pass

    @abstractmethod
    def process_action_str(self, action_line: str):
        pass

    @abstractmethod
    def parse_plan(self) -> tuple[bool, list]:
        pass

    def __get_domain_name(self, domain_file_name: str) -> str:
        '''Extracts the name of the planning domain from the given file
        by looking for the first line that contains the words "define" and
        "domain" and then parsing the domain line from it.

        Keyword arguments:
        @param domain_file_name: str -- absolute path of a PDDL domain file

        '''
        line = ''
        with open(domain_file_name, 'r') as domain_file:
            line = ''
            domain_defn_found = False
            while not domain_defn_found:
                line = domain_file.readline().lower()
                domain_defn_found = line.find('define') != -1 and line.find('domain') != -1
        domain_idx = line.lower().find('domain') + 1
        line = line[domain_idx+6:]
        idx = 0
        while line[idx] == ' ':
            idx += 1

        domain_name_start_idx = idx
        while line[idx] != ' ' and line[idx] != ')':
            idx += 1
        domain_name_end_idx = idx - 1
        domain_name = line[domain_name_start_idx:domain_name_end_idx+1]
        return domain_name
