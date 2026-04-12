from vlm_query_interface.vlm_interfaces import VLMFrameworks

class QueryInterfaceFactory(object):
    @staticmethod
    def get_vlm_interface(framework: str,
                          vlm_name: str,
                          system_prompt: tuple[str]):
        """Creates a VLM query interface instance from the desired framework.
        The allowed frameworks are defined in vlm_query_interface.vlm_interfaces.VLMFrameworks.
        Raises a ValueError if the framework is not one of the allowed values.

        Keyword arguments:
        framework: str -- The name of the model framework that should be used (such as ollama or transformers)
        vlm_name: str -- The name of the VLM model to use for queries
        system_prompt: tuple[str] -- The system prompt to use in queries

        """
        if framework == VLMFrameworks.ollama:
            from vlm_query_interface.vlm_interfaces.ollama import OllamaInterface
            return OllamaInterface(name=VLMFrameworks.ollama,
                                   vlm_name=vlm_name,
                                   system_prompt=system_prompt)
        elif framework == VLMFrameworks.transformers:
            from vlm_query_interface.vlm_interfaces.space_om import SpaceOmInterface
            return SpaceOmInterface(name=VLMFrameworks.transformers,
                                    vlm_name=vlm_name,
                                    system_prompt=system_prompt)
        else:
            raise ValueError(f'[get_vlm_interface] Received unknown framework name: {framework}')