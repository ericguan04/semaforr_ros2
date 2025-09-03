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
            'human_3d_lidar_fusion_node = social_context.pose_estimation.src.human_3d_lidar_fusion_node:main',
            'openpose_node = social_context.pose_estimation.src.openpose_node:main'
        ],
    },
)
