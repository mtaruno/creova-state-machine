from setuptools import setup

package_name = 'creova_state_machine'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.nodes'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Modular multi-node FSM architecture',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orchestration_node = creova_state_machine.nodes.orchestration_node:main',
            'navigation_node = creova_state_machine.nodes.navigation_node:main',
            'manipulation_node = creova_state_machine.nodes.manipulation_node:main',
            'perception_node = creova_state_machine.nodes.perception_node:main',
            'task_input_node = creova_state_machine.nodes.task_input_node:main',

        ],
    },
)