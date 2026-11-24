import base64
import numpy as np
import cv2
import anthropic

from vlm_query_interface.vlm_interfaces.vlm_base import VLMQueryInterfaceBase

class ClaudeInterface(VLMQueryInterfaceBase):
    """A query interface for Claude.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    def __init__(self, name: str,
                 vlm_name: str,
                 system_prompt: str):
        super(ClaudeInterface, self).__init__(name=name,
                                              system_prompt=system_prompt)
        self.vlm_name = vlm_name
        self.claude_client = anthropic.Anthropic()

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
        query_text = self.system_prompt + query

        response = None
        try:
            if image is not None:
                encoded_image = base64.b64encode(cv2.imencode('.jpg', image)[1].tobytes()).decode()
                response = self.claude_client.messages.create(model=self.vlm_name,
                                                              max_tokens=1000,
                                                              messages=[
                                                              {
                                                                    "role": "user",
                                                                    "content":
                                                                    [
                                                                        {
                                                                            "type": "image",
                                                                            "source": {"type": "base64",
                                                                                       "media_type": "image/jpeg",
                                                                                       "data": encoded_image}
                                                                        },
                                                                        {"type": "text", "text": query_text}
                                                                    ]
                                                              }])
            else:
                response = self.claude_client.messages.create(model=self.vlm_name,
                                                              max_tokens=1000,
                                                              messages=[
                                                              {
                                                                  "role": "user",
                                                                  "content": query_text
                                                              }])
            return (response.content[0].text, True)
        except Exception as exc:
            print(f'[{self.name}] Could not get Claude output; the following error occurred: {str(exc)}')
            return ('', False)