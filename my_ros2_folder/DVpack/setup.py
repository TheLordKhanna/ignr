from setuptools import find_packages, setup

#all executable files need to be declared in setup.py

package_name = 'DVpack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
        'launch/dvpack_bridge.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2box',
    maintainer_email='ros2box@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = DVpack.publisher_member_function:main',
            'listener = DVpack.subscriber_member_function:main',
            'transform_subscriber_notworking = DVpack.transform_subscriber:main',
            'secondtrynode = DVpack.secondtrynode:main',
            'trial_subscriber = DVpack.trialsubscriber:main',
            'another_trial = DVpack.anothertrial:main',
            'moveit_commanderfile = DVpack.moveitcommanderfile:main',
        ],
    },
)
