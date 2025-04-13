"""
Contains helper functions for:
- Preprocessing raw data
- Running the model
- Converting model outputs for trajectory prediction in multiple environments

Functions:
- preprocess_data(raw_data):
    Preprocesses the raw data into input tensors for the model.

- predict(obs_seq_len, pred_seq_len, input_traj, input_binary_mask, load_path, args_path):
    Runs the model to predict future trajectories based on observed trajectories.

    Inputs:
    - obs_seq_len (int): The length of the observed sequence.
    - pred_seq_len (int): The length of the predicted sequence.
    - input_traj (torch.Tensor): The input trajectory tensor of shape (n_env, num_peds, obs_seq_len, 2).
    - input_binary_mask (torch.Tensor): The input binary mask tensor of shape (n_env, num_peds, obs_seq_len, 1).
    - load_path (str): The path to the model checkpoint.
    - args_path (str): The path to the model arguments file.

    Outputs:
    - output_traj (torch.Tensor): The output trajectory tensor of shape (n_env, num_peds, pred_seq_len, 5).
    - output_binary_mask (torch.Tensor): The output binary mask tensor of shape (n_env, num_peds, pred_seq_len, 1).

- output_traj_to_dict(output_traj, id_to_index):
    Converts the output trajectory tensor back into a dictionary format.
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
import pickle
from . wrapper import CrowdNavPredInterfaceMultiEnv

# Example raw data with multiple time steps for each environment
# raw_data = [
#     # Env 0
#     [
#         {'id': 'A', 'coords': [(1.0, 2.0), (1.1, 2.1), (1.2, 2.2)]},
#         {'id': 'B', 'coords': [(2.5, 3.5), (2.6, 3.6), (2.7, 3.7)]}
#     ],
#     # Env 1
#     [
#         {'id': 'A', 'coords': [(1.2, 2.1), (1.3, 2.2), (1.4, 2.3)]},
#         {'id': 'B', 'coords': [(2.6, 3.6), (2.7, 3.7), (2.8, 3.8)]}
#     ],
#     # Env 2
#     [
#         {'id': 'A', 'coords': [(1.1, 2.2), (1.2, 2.3), (1.3, 2.4)]},
#         {'id': 'B', 'coords': [(2.7, 3.7), (2.8, 3.8), (2.9, 3.9)]}
#     ],
#     # Env 3
#     [
#         {'id': 'A', 'coords': [(1.3, 2.3), (1.4, 2.4), (1.5, 2.5)]},
#         {'id': 'B', 'coords': [(2.8, 3.8), (2.9, 3.9), (3.0, 4.0)]}
#     ]
# ]

def preprocess_data(raw_data):
    """
    Preprocess the raw data into input tensors for the model.
    
    Args:
    - raw_data (list): A list of environments, where each environment is a list of pedestrians.
    
    Returns:
    - torch.Tensor: The input trajectory tensor of shape (n_env, num_peds, obs_seq_len, 2).
    - torch.Tensor: The input binary mask tensor of shape (n_env, num_peds, obs_seq_len, 1).
    - dict: A dictionary mapping pedestrian IDs to their indices.
    """
    # Step 1: Create ID to index mapping (Assuming IDs are unique across all environments)
    unique_ids = sorted(set(person['id'] for env in raw_data for person in env))
    id_to_index = {id_: idx for idx, id_ in enumerate(unique_ids)}

    # Initialize tensor dimensions
    number_of_env = len(raw_data)
    number_of_pedestrians = len(unique_ids)  # Total unique pedestrians
    number_of_time_steps = len(raw_data[0][0]['coords'])  # Time steps per pedestrian

    # Initialize tensors
    input_traj = torch.zeros((number_of_env, number_of_pedestrians, number_of_time_steps, 2))
    input_binary_mask = torch.zeros((number_of_env, number_of_pedestrians, number_of_time_steps, 1))

    # Step 2: Fill tensors based on raw data using the ID mapping
    for env_idx, env_data in enumerate(raw_data):
        for person in env_data:
            id_ = person['id']
            coords = person['coords']

            # Find index using the ID mapping
            ped_idx = id_to_index[id_]

            # Fill in the coordinates and set the binary mask to 1.0 for this person at each time step
            for t, coord in enumerate(coords):
                input_traj[env_idx, ped_idx, t, :2] = torch.tensor(coord)
                input_binary_mask[env_idx, ped_idx, t, 0] = 1.0

    return input_traj, input_binary_mask, id_to_index


def predict(obs_seq_len,
            pred_seq_len, 
            input_traj, 
            input_binary_mask,
            load_path,
            args_path):
    
    n_env = input_traj.shape[0]
    assert input_traj.shape[2] == obs_seq_len
  
    """
    - input_traj:
        # tensor
        # (n_env, num_peds, obs_seq_len, 2)
    - input_binary_mask:
        # tensor
        # (n_env, num_peds, obs_seq_len, 1)
    """
    print()
    print("INPUT DATA")
    print("number of environments: ", n_env)
    print("input_traj shape: ", input_traj.shape)
    print("input_binary_mask shape:", input_binary_mask.shape)
    print()

    #load_path = '/Users/ericguan/Documents/CrowdNav_Prediction_AttnGraph/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj'
    #device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    device = torch.device("cpu")
    # Load args from a pickle file

    # args_path = '/Users/ericguan/Documents/CrowdNav_Prediction_AttnGraph/gst_updated/results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/args.pickle'
    with open(args_path, 'rb') as f:
        args = pickle.load(f)
    args.obs_seq_len = 3
    args.pred_seq_len = 3
    print(args)
    model = CrowdNavPredInterfaceMultiEnv(load_path=load_path,
                                            device=device, config = args, num_env=n_env)

    input_traj = input_traj.cpu()
    input_binary_mask = input_binary_mask.cpu()
    output_traj, output_binary_mask = model.forward(
        input_traj,
        input_binary_mask,
        sampling = True,
    )
    
    return output_traj, output_binary_mask


def output_traj_to_dict(output_traj, id_to_index):
    """
    Convert the output trajectory tensor back into a dictionary format.
    
    Args:
    - output_traj (torch.Tensor): The output trajectory tensor of shape (n_env, num_peds, pred_seq_len, 5).
    - id_to_index (dict): A dictionary mapping pedestrian IDs to their indices.
    
    Returns:
    - dict: A dictionary where keys are pedestrian IDs and values are lists of (x, y) coordinates.
    """
    index_to_id = {v: k for k, v in id_to_index.items()}
    n_env, num_peds, pred_seq_len, _ = output_traj.shape
    output_dict = {index_to_id[i]: [] for i in range(num_peds)}

    for env_idx in range(n_env):
        for ped_idx in range(num_peds):
            for t in range(pred_seq_len):
                x, y = output_traj[env_idx, ped_idx, t, :2].tolist()
                output_dict[index_to_id[ped_idx]].append((x, y))
    
    return output_dict
