import base64
import cv2
import numpy as np
import torch
from transformers import Qwen2_5_VLForConditionalGeneration, AutoProcessor
from bs4 import BeautifulSoup

from vlm_query_interface.vlm_interfaces.vlm_base import VLMQueryInterfaceBase

class SpaceOmInterface(VLMQueryInterfaceBase):
    """A query interface for SpaceOm.

    @author Alex Mitrevski
    @contact alemitr@chalmers.se

    """
    def __init__(self, name: str,
                 vlm_name: str,
                 system_prompt: str):
        super(SpaceOmInterface, self).__init__(name=name,
                                               system_prompt=system_prompt)
        self.vlm_name = vlm_name
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(self.vlm_name,
                                                                        torch_dtype=torch.bfloat16)
        self.processor = AutoProcessor.from_pretrained(self.vlm_name)

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

        if encoded_image is not None:
            chat = [{"role": "system", "content": [{"type": "text", "text": self.system_prompt}]},
                    {"role": "user", "content": [{"type": "image", "image": encoded_image},
                                                 {"type": "text", "text": query}]}
                ]
            text_input = self.processor.apply_chat_template(chat,
                                                            tokenize=False,
                                                            add_generation_prompt=True)
            inputs = self.processor(text=[text_input],
                                    images=[encoded_image],
                                    return_tensors="pt")
        else:
            chat = [{"role": "system", "content": [{"type": "text", "text": self.system_prompt}]},
                    {"role": "user", "content": [{"type": "text", "text": query}]}
                ]
            text_input = self.processor.apply_chat_template(chat,
                                                            tokenize=False,
                                                            add_generation_prompt=True)
            inputs = self.processor(text=[text_input],
                                    return_tensors="pt")

        generated_ids = self.model.generate(**inputs, max_new_tokens=1024)
        output = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

        # we remove the prompt from the output
        prompt_length = len(query) + sum([len(x) for x in self.system_prompt])
        output = output[prompt_length:]

        answer = output[output.find('<answer>'):]
        answer_xml = BeautifulSoup(answer, 'lxml')
        if answer_xml is None:
            print(f'[{self.name}] Ignoring response {output} as it is not in the expected format')
            return ('', False)

        answer_entry = answer_xml.find('answer')
        if answer_entry is None:
            print(f'[{self.name}] Ignoring response {output} as it is not in the expected format')
            return ('', False)

        response_str = answer_entry.contents[0].strip().rstrip()
        return (response_str, True)