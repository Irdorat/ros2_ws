from setuptools import find_packages, setup

package_name = 'my_first_pkg'

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
    maintainer='ppk',
    maintainer_email='ppk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.talker:main',
            'listener = my_first_pkg.listener:main',
            'local_dsg = my_first_pkg.local_dsg:main',
            'capture_episode = my_first_pkg.capture_episode:main',
            'make_test_sensors = my_first_pkg.make_test_sensors:main',
            'publisher = my_first_pkg.publisher:main',
            'subscriber = my_first_pkg.subscriber:main',
            'service = my_first_pkg.service_member_function:main',
            'client = my_first_pkg.client_member_function:main',
            'launch = my_first_pkg.launch_mimic_turtle:main',
        ],
    },
)
