"""
Complete pipeline for trajectory prediction

Input:
    raw_data: The recorded trajectories of each pedestrian in (x, y) coordinate format
    obs_seq_len: The number of time steps to observe before making a prediction
    pred_seq_len: The number of time steps to predict into the future

Output:
    traj_preds: The predicted trajectories for each pedestrian

"""

# Resolve Pathing Issues
# import sys
# sys.path.append('/Users/ericguan/Documents/semaforr_ros2/social_context/social_context/trajectory_prediction')
# sys.path.insert(0, '/root/semaforr_ros2/src/social_context/social_context/trajectory_prediction')
from . utils import *
from . wrapper import *

def pipeline(raw_data, obs_seq_len, pred_seq_len):
    input_traj, input_binary_mask, id_to_index = preprocess_data(raw_data)

    # MacOS
    # load_path = '/Users/ericguan/Documents/semaforr_ros2/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj'
    # args_path = '/Users/ericguan/Documents/semaforr_ros2/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/args.pickle'

    # Linux
    load_path = '/root/semaforr_ros2/src/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj'
    args_path = '/root/semaforr_ros2/src/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/args.pickle'

    output_traj, output_binary_mask = predict(obs_seq_len,
                                              pred_seq_len, 
                                              input_traj, 
                                              input_binary_mask,
                                              load_path,
                                              args_path)

    traj_preds = output_traj_to_dict(output_traj, id_to_index)

    return traj_preds