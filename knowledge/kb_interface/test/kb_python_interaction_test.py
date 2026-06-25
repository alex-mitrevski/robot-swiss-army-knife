from kb_interface.knowledge_base_interface import KnowledgeBaseInterface

def print_assertions(fluent_assertions):
    for fluent in fluent_assertions:
        fluent_string = f'{fluent.name} -- '
        param_string = ''
        for param in fluent.params:
            param_string += f'({param.name}, {param.value}) '
        fluent_string += f'[ {param_string}] -- {fluent.value}'
        print(fluent_string)
    print()

if __name__ == '__main__':
    kb_interface = KnowledgeBaseInterface('robot_store')

    print('Retrieving assertions before insertion')
    fluent_assertions = kb_interface.get_fluent_assertions()
    print_assertions(fluent_assertions)

    state_fluents = [('robotName', [('Robot', 'MyRobot')], 'true'),
                        ('robotAt', [('Robot', 'MyRobot')], 'Kitchen')]

    print('Inserting assertions')
    kb_interface.insert_fluents(state_fluents)

    print('Retrieving assertions after insertion')
    fluent_assertions = kb_interface.get_fluent_assertions()
    print_assertions(fluent_assertions)

    print('Retrieving assertions of type robotAt')
    fluent_assertions = kb_interface.get_fluent_assertions('robotAt')
    print_assertions(fluent_assertions)

    print('Removing assertions')
    kb_interface.remove_fluents(state_fluents)

    print('Retrieving assertions after removal')
    fluent_assertions = kb_interface.get_fluent_assertions()
    print_assertions(fluent_assertions)