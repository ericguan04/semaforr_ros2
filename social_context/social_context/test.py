from trajectory_prediction.pipeline import pipeline

social_context_model = {
    'A': {
        'history': [(1.0, 2.0), (1.1, 2.1), (1.2, 2.2)],
        'predictions': []
    },
    'B': {
        'history': [(2.5, 3.5), (2.6, 3.6), (2.7, 3.7)],
        'predictions': []
    }
}

def social_context_to_raw_data(social_context_model):
    raw_data = []
    for key, value in social_context_model.items():
        raw_data.append({'id': key, 'coords': value['history']})
    return [raw_data]

raw_data = social_context_to_raw_data(social_context_model)

print(raw_data)

# Trajectory prediction pipeline works with a single environment
# Experiment with multiple environments (computational cost vs accuracy tradeoff)

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

# print()
# print(raw_data)

traj_preds = pipeline(raw_data, 3, 3)

print(traj_preds)

print()

for key, value in traj_preds.items():
    print(f"Pedestrian {key}:")
    for coord in value:
        print(f"  - {coord}")