from setuptools import find_packages, setup

package_name = 'social_context'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # find_packages()/colcon only copy .py files by default -- the GST model's
    # checkpoint (loaded via a path relative to pipeline.py at runtime) needs
    # to be explicitly included so it actually lands in the installed package,
    # not just the source tree.
    package_data={
        'social_context.trajectory_prediction.gst_updated': [
            'results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/*.pt',
            'results/100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000/sj/checkpoint/*.pickle',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'social_context_hunav = social_context.social_context_hunav:main',
            'social_context_tracked = social_context.social_context_tracked:main',
            'image_publisher = social_context.pose_estimation.tests.image_publisher:main',
            'tf_human_matcher = social_context.pose_estimation.tests.tf_human_matcher:main',
            'person_relative_localizer = social_context.pose_estimation.src.person_relative_localizer:main',
            # 'openpose_node = social_context.pose_estimation.src.openpose_node:main',
            # 'mediapipe_pose_node = social_context.pose_estimation.src.mediapipe_pose_node:main',
            'pose_openpose = social_context.pose_estimation.src.camera_2d_pose_detection_node:main_openpose',
            'pose_mediapipe = social_context.pose_estimation.src.camera_2d_pose_detection_node:main_mediapipe',
            'global_human_localizer = social_context.pose_estimation.src.global_human_localizer:main',
            'sort_tracker = social_context.pose_estimation.src.tracking.sort_node:main',
            'tracking_evaluator = social_context.pose_estimation.src.tracking.testing.tracking_evaluator:main',
        ],
    },
)
