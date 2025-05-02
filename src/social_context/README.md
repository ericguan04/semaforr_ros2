# Social_Context Package
This package consists of all the nodes, functions, and models needed for the social context model used by the SemaFORR navigation system

How to use:
* `ros2 run social_context (node)`

Available Nodes:
* `social_context_hunav`: This node takes in /human_states topic from HuNavSim, runs the deep learning prediction function, updates the data structure, and sends out the predicted pose. It subscribes to the Agents message and publishes PoseStamped message

Virtual Environment:
* Running the social context model requires certain dependencies for the models to run. It is recommended to set up a virtual environment (like conda) with Python 3.10. Then install the dependencies from requirements.txt in /social_context/trajectory_prediction (pip install -r requirements.txt)