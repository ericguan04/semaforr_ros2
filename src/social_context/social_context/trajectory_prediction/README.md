Code in this folder is based on code from the following papers:

* Liu, S., Chang, P., Huang, Z., Chakraborty, N., Hong, K., Liang, W., ... & Driggs-Campbell, K. (2023, May). Intention aware robot crowd navigation with attention-based interaction graph. In 2023 IEEE International Conference on Robotics and Automation (ICRA) (pp. 12015-12021). IEEE. (Github: https://github.com/Shuijing725/CrowdNav_Prediction_AttnGraph)

* Z. Huang, R. Li, K. Shin, and K. Driggs-Campbell. "Learning Sparse Interaction Graphs of Partially Detected Pedestrians for Trajectory Prediction," in IEEE Robotics and Automation Letters, vol. 7, no. 2, pp. 1198–1205, 2022. (Github: https://github.com/tedhuang96/gst)

High level functions:
* `pipeline.py`: combines all the functions from `utils.py` and `wrapper.py` into one end-to-end function for trajetory prediction
* `utils.py`: contains functions for preprocessing the raw data, predicting, and processing the output data 
* `wrapper.py`: contains preprocessing functions used by the deep learning model

See `test.py` and `example_run.ipynb` for examples of these functions being used