from setuptools import find_packages, setup

package_name = 'social_context'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'image_publisher = social_context.pose_estimation.tests.image_publisher:main',
            'person_relative_localizer = social_context.pose_estimation.src.person_relative_localizer:main',
            'openpose_node = social_context.pose_estimation.src.openpose_node:main',
            # 'mediapipe_pose_node = social_context.pose_estimation.src.mediapipe_pose_node:main',
            'pose_mediapipe = social_context.pose_estimation.src.camera_2d_pose_detection_node:main_mediapipe',
        ],
    },
)
