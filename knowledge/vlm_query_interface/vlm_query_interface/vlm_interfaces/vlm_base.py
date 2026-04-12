import abc
import numpy as np

class VLMQueryInterfaceBase(object):
    """A base class for VLM query interfaces.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    def __init__(self, name: str,
                 system_prompt: str):
        self.name = name
        self.system_prompt = system_prompt

    @abc.abstractmethod
    def query_model(self, query: str, image: np.ndarray) -> str:
        """An abstract method for querying a VLM.

        Keyword arguments:
        query: str -- Query text
        image: np.ndarray -- Query image

        """
        raise NotImplementedError(f'[{self.name}] query_model is not implemented')
