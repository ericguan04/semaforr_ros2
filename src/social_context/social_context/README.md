# Social Context Package Nodes

## `camera_listener.py` 
* `CameraListener(Node)`: Subscribes to sensor data, processes the data, and publishes to social context model node
* Names subject to change

## `social_context.py`
* `CoordinateListener(Node)`: A ROS2 node that subscribes to a topic to receive pedestrian coordinates, updates a social context model, and makes trajectory predictions.
* `social_context_model`: A dictionary containing pedestrian IDs, coordinates, and predictions. Used by SemaFORR reasoning and decision making architecture