from termcolor import colored

from kb_interface.knowledge_base_interface import KnowledgeBaseInterface

if __name__ == '__main__':
    kb_interface = KnowledgeBaseInterface('robot_store')
    try:        
        kb_interface.clear_kb()
        print(colored('Knowledge base cleared', 'green'))
    except Exception as exc:
        print(colored(f'Could not clear knowledge base: {str(exc)}', 'red'))
