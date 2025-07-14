from setuptools import find_packages, setup

package_name = 'rokey_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rokey_project/sounds', ['sounds/Have_a_nice_day.wav', 'sounds/Request_Complete.wav']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feedback_node = rokey_project.feedback_node:main',
            'robot_control_node = rokey_project.robot_control_node:main',
            'test = rokey_project.publish_test_node:main',
        ],
    },
)
