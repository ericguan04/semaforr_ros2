# Social_ Context Package Nodes
This section dives deeper into the code implementation of the nodes

## `social_context_hunav`
* `CoordinateListener(Node)`: subscribes to message type Agents and topic name /human_states. This is the human states being published by HuNavSim. It publishes message type PoseStamped and topic name /pedestrian_predictions. In between, the node makes predictions from the subscribed data and updates the social context model (hashmap data structure)

--- Work in Progress Below (subject to change) ---

## `camera_listener.py` 
* `CameraListener(Node)`: Subscribes to sensor data, processes the data, and publishes to social context model node
* Names subject to change

## `social_context.py`
* `CoordinateListener(Node)`: A ROS2 node that subscribes to a topic to receive pedestrian coordinates, updates a social context model, and makes trajectory predictions.
* `social_context_model`: A dictionary containing pedestrian IDs, coordinates, and predictions. Used by SemaFORR reasoning and decision making architecture