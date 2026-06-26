import os
from os import listdir
from os.path import join
import uuid
import subprocess
import numpy as np

from task_planner_interface.planner_interfaces.planner_interface_base import TaskPlannerInterfaceBase
from kb_interface.knowledge_base_interface import Fluent

class LAMAInterface(TaskPlannerInterfaceBase):
    _plan_file_name = 'plan.txt'

    def __init__(self, kb_database_name, domain_file,
                 planner_cmd, plan_file_path, debug=False):
        super(LAMAInterface, self).__init__(kb_database_name, domain_file,
                                            planner_cmd, plan_file_path,
                                            debug)

    def plan(self):
        task_goals = self.kb_interface.get_goals()
        kb_fluent_assertions = self.kb_interface.get_fluent_assertions()

        print('Generating problem file')
        problem_file = self.generate_problem_file(kb_fluent_assertions, task_goals)

        planner_cmd = self.planner_cmd.replace('PROBLEM', problem_file)
        planner_cmd = planner_cmd.replace('PLAN-FILE', join(self.plan_file_path,
                                                            self._plan_file_name))
        planner_cmd_elements = planner_cmd.split()

        print('Planning task...')
        subprocess.run(planner_cmd_elements)
        print('Planning finished')

        print('Parsing plans...')
        plan_found, plan = self.parse_plan()

        print('Removing problem file...')
        os.remove(problem_file)
        print('Planner done')

        return plan_found, plan

    def generate_problem_file(self, fluent_assertions: list,
                              task_goals: list[Fluent]) -> str:
        obj_types = {}
        init_state_str = ''

        # for numeric fluents, we generate strings of the form
        # (= (fluent_name param_1 param_2 ... param_n) fluent_value);
        # otherwise, we generate strings of the form
        # (predicate_name param_1 param_2 ... param_n)
        for assertion in fluent_assertions:
            if hasattr(self.fluent_lib, assertion.name):
                ordered_param_list, obj_types = self.fluent_lib.get_assertion_param_list(assertion.name,
                                                                                         assertion.params,
                                                                                         assertion.value,
                                                                                         obj_types)

                # if the fluent assertion contains a value that is not of Boolean type,
                # we explicitly add the value to the assertion string; otherwise,
                # we just add the fluent parameters and ensure that the assertion
                # corresponds to the asserted truth value
                if assertion.value.lower() != 'true' and assertion.value.lower() != 'false':
                    assertion_str = '        ({0} {1} {2})\n'.format(assertion.name,
                                                                    ' '.join(ordered_param_list),
                                                                    assertion.value)
                elif assertion.value.lower() == 'true':
                    assertion_str = '        ({0} {1})\n'.format(assertion.name,
                                                                 ' '.join(ordered_param_list))
                elif assertion.value.lower() == 'false':
                    assertion_str = '        not ({0} {1})\n'.format(assertion.name,
                                                                     ' '.join(ordered_param_list))
            else:
                ordered_param_list, obj_types = self.numeric_fluent_lib.get_assertion_param_list(assertion.name,
                                                                                                 assertion.params,
                                                                                                 obj_types)
                assertion_str = '        (= ({0} {1}) {2})\n'.format(assertion.name,
                                                                     ' '.join(ordered_param_list),
                                                                     assertion.value)
            init_state_str += assertion_str

        # we combine the assertion strings into an initial state string of the form
        # (:init
        #     assertions
        # )
        init_state_str = '    (:init\n{0}\n    )\n\n'.format(init_state_str)

        # we generate a string with the object types of the form
        # (:objects
        #     obj_11 obj_12 - type_1
        #     ...
        #     obj_n1 - type_n
        # )
        obj_type_str = ''
        for obj_type in obj_types:
            obj_type_str += '        {0} - {1}\n'.format(' '.join(obj_types[obj_type]), obj_type)
        obj_type_str = '    (:objects\n{0}    )\n\n'.format(obj_type_str)

        # we generate a string with the planning goals of the form
        # (:goals
        #     (and
        #         (predicate_1_name param_1 param_2 ... param_n)
        #         ...
        #         (predicate_n_name param_1 param_2 ... param_n)
        #     )
        # )
        goal_str = ''
        for task_goal in task_goals:
            if task_goal.value.lower() != 'true' and task_goal.value.lower() != 'false':
                goal_str += '            ({0} {1} {2})\n'.format(task_goal.name,
                                                                 ' '.join([param.value for param in task_goal.params]),
                                                                 task_goal.value)
            elif task_goal.value.lower() == 'true':
                goal_str += '        ({0} {1})\n'.format(task_goal.name,
                                                         ' '.join([param.value for param in task_goal.params]))
            elif task_goal.value.lower() == 'false':
                assertion_str += '        not ({0} {1})\n'.format(task_goal.name,
                                                                  ' '.join([param.value for param in task_goal.params]))
        goal_str = '    (:goal\n        (and\n{0}        )\n    )\n'.format(goal_str)

        # we finally write the problem file, which will be in the format
        # (define (problem problem-name)
        #     (:domain domain-name)
        #     (:objects
        #         ...
        #     )
        #     (:objects
        #         ...
        #     )
        #     (:goals
        #         ...
        #     )
        # )
        problem_file_name = 'problem_{0}.pddl'.format(str(uuid.uuid4()))
        problem_file_abs_path = join(self.plan_file_path, problem_file_name)
        print('Generating planning problem...')
        with open(problem_file_abs_path, 'w') as problem_file:
            header = '(define (problem ropod)\n'
            header += '    (:domain {0})\n'.format(self.domain_name)
            problem_file.write(header)
            problem_file.write(obj_type_str)
            problem_file.write(init_state_str)
            problem_file.write(goal_str)
            problem_file.write(')\n')
        return problem_file_abs_path

    def parse_plan(self) -> tuple[bool, list]:
        plan_files = [f for f in listdir(self.plan_file_path)
                      if f.find(self._plan_file_name) != -1]
        if not plan_files:
            print(f'Plan could not be found')
            return False, []

        plans = []
        action_strings_per_plan = []
        plan_counter = 1
        for plan_file_name in plan_files:
            if self.debug:
                print(f'Processing plan {plan_counter} out of {len(plan_files)}')

            plan = []
            plan_action_strings = []
            current_plan_file_path = join(self.plan_file_path, plan_file_name)
            with open(current_plan_file_path, 'r') as plan_file:
                while True:
                    line = plan_file.readline()
                    if line.find(';') != -1:
                        break
                    else:
                        action_line = line.strip()[1:-1]
                        action = self.process_action_str(action_line)
                        plan.append(action)
                        plan_action_strings.append(action_line)
                        if self.debug:
                            print(f'Action line from plan: {action_line}')
                plans.append(plan)
                action_strings_per_plan.append(plan_action_strings)
            os.remove(current_plan_file_path)
            plan_counter += 1
            if self.debug:
                print()

        plan_lengths = [len(plan) for plan in plans]
        shortest_plan_idx = np.argmin(plan_lengths)

        print(f'Plan found')
        print('Action sequence:')
        print('-------------------------------')
        for action_string in action_strings_per_plan[shortest_plan_idx]:
            print(action_string)
        print('-------------------------------')
        return True, plans[shortest_plan_idx]

    def process_action_str(self, action_line: str):
        action_data = action_line.split()
        action_name = action_data[0].upper()
        action_params = action_data[1:]
        action = self.action_model_lib.get_action_model(action_name, action_params)
        if self.debug:
            print(f'Grounded action: {action}')
        return action
