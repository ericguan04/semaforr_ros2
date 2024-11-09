from trajectory_prediction.pipeline import pipeline

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

traj_preds = pipeline(raw_data, 3, 3)

for key, value in traj_preds.items():
    print(f"Pedestrian {key}:")
    for coord in value:
        print(f"  - {coord}")