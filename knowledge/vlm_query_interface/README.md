# vlm_query_interface

A Python-based package that exposes an interface for sending VLMs / LLMs queries. Currently, only models available through [Ollama](https://ollama.com) as well as [SpaceOm](https://huggingface.co/remyxai/SpaceOm) are exposed.

## Dependencies

[robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs) is a common dependency.

The component also has conditional dependencies, depending on which model framework is used:
* If only the Ollama models are needed, then [Ollama](https://ollama.com) is a dependency.
* If SpaceOm is used, the dependencies are as follows:
    * [transformers](https://pypi.org/project/transformers/)
    * [PyTorch](https://pytorch.org)
    * [beautifulsoup4](https://pypi.org/project/beautifulsoup4/)

## Usage instructions

The component includes a node that exposes an action for processing queries. After building this package, the node can be started as follows:
```
ros2 launch vlm_query_interface query_interface.launch.py
```

The action interface used by the component is of type [robot_swiss_knife_msgs/action/QueryVLM](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/action/QueryVLM.action). While I developed the component primarily for querying VLMs, if an empty image is sent in the action request, a pure textual query will be sent to the model; thus, LLMs are also supported by the component, not just VLMs.

The action name (`/query_vlm` by default) can be specified in the launch file. Three other optional parameters can also be specified:
* `vlm_framework_name` (default value `transformers`): Currently, `ollama` and `transformers` are supported values.
* `vlm_name` (default value `remyxai/SpaceOm`): If `ollama` is used as a framework, any Ollama-supported model name can be specified. If `transformers` is used as a framework, `remyxai/SpaceOm` should be passed as a value (in future, I might extend the component to support other models as well).
* `system_prompt` (default value `""`): Any system prompt that should be sent with all VLM / LLM queries.