import base64
import numpy as np
import cv2
import ollama

from vlm_query_interface.vlm_interfaces.vlm_base import VLMQueryInterfaceBase

class OllamaInterface(VLMQueryInterfaceBase):
    """A query interface for a model exposed through Ollama.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    def __init__(self, name: str,
                 vlm_name: str,
                 system_prompt: str):
        super(OllamaInterface, self).__init__(name=name,
                                              system_prompt=system_prompt)
        self.vlm_name = vlm_name
        self.ollama_client = ollama.Client()

    def query_model(self, query: str, image: np.ndarray) -> tuple[str, bool]:
        """Queries the model with the given query text and image.
        If the image is None, sends a pure textual model query.

        Keyword arguments:
        query: str -- Query text
        image: np.ndarray -- Query image

        Returns:
        response: str -- The query response
        success: bool -- If False, indicates that an error occurred
                         while the query was being processed

        """
        encoded_image = None
        if image is not None:
            encoded_image = base64.b64encode(cv2.imencode('.jpg', image)[1].tobytes()).decode()
        query_text = self.system_prompt + query

        response_gen = None
        if encoded_image is not None:
            response_gen = self.ollama_client.chat(model=self.vlm_name,
                messages=[
                {
                    'role': 'user',
                    'content': query_text,
                    'images': [encoded_image]
                }],
                stream=True)
        else:
            response_gen = self.ollama_client.chat(model=self.vlm_name,
                messages=[
                {
                    'role': 'user',
                    'content': query_text
                }],
                stream=True)

        response_str = ''
        try:
            for part in response_gen:
                response_str += part['message']['content']
        except:
            print('Invalid response received; skipping')
            return (response_str, False)

        return (response_str, True)