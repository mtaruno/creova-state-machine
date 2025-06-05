from setuptools import setup
import os
from glob import glob

package_name = 'creova_state_machine'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.test'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gixstudent',
    maintainer_email='gixstudent@todo.todo',
    description='Voice-controlled delivery robot system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = creova_state_machine.nodes.perception_node:main',
            'navigation_node = creova_state_machine.nodes.navigation_node:main',
            'orchestration_node = creova_state_machine.nodes.orchestration_node:main',
            'manipulation_node = creova_state_machine.nodes.manipulation_node:main',
            'navigation_tester = creova_state_machine.test.test_navigation:main',
            'physical_ai_node = creova_state_machine.nodes.physical_ai_node:main',
            'get_destination_client = creova_state_machine.nodes.get_destination_client:main',
            'destination_server_node = creova_state_machine.nodes.destination_server_node:main',
            'test_destination_server = creova_state_machine.nodes.test_destination_server:main',
            'get_destination_server = creova_state_machine.nodes.get_destination_server:main',
            'final_detection_subscriber = creova_state_machine.nodes.final_detection_subscriber:main',
            'publisher = creova_state_machine.nodes.publisher:main',
            'status_logger_node = creova_state_machine.nodes.status_logger_node:main',
            'status_replayer_node = creova_state_machine.nodes.status_replayer_node:main',
            'nav_ready_trigger = creova_state_machine.nodes.nav_ready_trigger:main',   
            'state_monitor_node = creova_state_machine.nodes.state_monitor_node:main',
            'test_orchestration = creova_state_machine.test.test_orchestration:main',

        ],
    },
)