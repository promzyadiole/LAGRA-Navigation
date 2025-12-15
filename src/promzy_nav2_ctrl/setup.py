from setuptools import setup
from glob import glob
import os

package_name = 'promzy_nav2_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # uses promzy_nav2_ctrl/__init__.py
    data_files=[
        # ROS 2 package index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),

        # Package manifest
        (os.path.join('share', package_name), ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # Install config files from the sub-package folder (promzy_nav2_ctrl/promzy_nav2_ctrl/config)
        ('share/' + package_name + '/config', glob('promzy_nav2_ctrl/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Promise',
    author_email='you@example.com',
    maintainer='Promise',
    maintainer_email='you@example.com',
    description='Nav2 commander + LLM bridge (ROS 2 Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_test = promzy_nav2_ctrl.nav2_test:main',
            'llm_command = promzy_nav2_ctrl.llm_command:main',
            'nl_command_pub = promzy_nav2_ctrl.nl_command_pub:main',
        ],
    },
)
