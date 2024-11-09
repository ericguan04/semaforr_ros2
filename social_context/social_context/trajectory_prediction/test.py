from utils import *
from wrapper import *

"""
Get the raw data from another ros2 node
Maybe make a function that will convert raw data into the format below

Process raw trajectory data -> predict future trajectories -> convert predicted trajectories to dictionary format
"""

raw_data = [
    # Env 0
    [
        {'id': 'A', 'coords': [(1.0, 2.0), (1.1, 2.1), (1.2, 2.2)]},
        {'id': 'B', 'coords': [(2.5, 3.5), (2.6, 3.6), (2.7, 3.7)]}
    ],
    # Env 1
    [
        {'id': 'A', 'coords': [(1.2, 2.1), (1.3, 2.2), (1.4, 2.3)]},
        {'id': 'B', 'coords': [(2.6, 3.6), (2.7, 3.7), (2.8, 3.8)]}
    ],
    # Env 2
    [
        {'id': 'A', 'coords': [(1.1, 2.2), (1.2, 2.3), (1.3, 2.4)]},
        {'id': 'B', 'coords': [(2.7, 3.7), (2.8, 3.8), (2.9, 3.9)]}
    ],
    # Env 3
    [
        {'id': 'A', 'coords': [(1.3, 2.3), (1.4, 2.4), (1.5, 2.5)]},
        {'id': 'B', 'coords': [(2.8, 3.8), (2.9, 3.9), (3.0, 4.0)]}
    ]
]

input_traj, input_binary_mask, id_to_index = preprocess_data(raw_data)

# Check results
print("input_traj:", input_traj)
print("input_binary_mask:", input_binary_mask)
print("ID to index mapping:", id_to_index)
print(input_traj.shape)

obs_seq_len = 3
pred_seq_len = 3
load_path = '/Users/ericguan/Documents/semaforr_ros2/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj'
args_path = '/Users/ericguan/Documents/semaforr_ros2/social_context/social_context/trajectory_prediction/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/args.pickle'

output_traj, output_binary_mask = predict(obs_seq_len,
            pred_seq_len, 
            input_traj, 
            input_binary_mask,
            load_path,
            args_path)

print()
print("OUTPUT DATA")
print("output_traj shape: ", output_traj.shape)
print("output_binary_mask shape:", output_binary_mask.shape)
print()
print(output_traj)

traj_preds = output_traj_to_dict(output_traj, id_to_index)

for key, value in traj_preds.items():
    print(f"Pedestrian {key}:")
    for coord in value:
        print(f"  - {coord}")