import os
import time
from termcolor import colored

from kb_interface.knowledge_base_interface import KnowledgeBaseInterface

def print_assertions(fluent_assertions):
    for fluent in fluent_assertions:
        fluent_string = f'{fluent.name} -- '
        param_string = ''
        for param in fluent.params:
            param_string += f'({param.name}, {param.value}) '
        fluent_string += f'[ {param_string}] -- {fluent.value}'
        print(colored(fluent_string, 'green'))
    print()

if __name__ == '__main__':
    kb_interface = KnowledgeBaseInterface('robot_store')
    try:
        while True:
            fluent_assertions = kb_interface.get_fluent_assertions()
            print_assertions(fluent_assertions)
            time.sleep(1.)
            os.system('clear')
    except (KeyboardInterrupt, SystemExit):
        print('Ending knowledge base visualiser')
