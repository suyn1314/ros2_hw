from setuptools import setup
import os
from glob import glob
package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuchenck',
    maintainer_email='yuchenck@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                 'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',
        	 'turtle_tf2_message_broadcaster = learning_tf2_py.turtle_tf2_message_broadcaster:main',
        	 'turtle_tf2_message_broadcaster_1 = learning_tf2_py.turtle_tf2_message_broadcaster_1:main',
        	 'turtle_tf2_message_broadcaster_2 = learning_tf2_py.turtle_tf2_message_broadcaster_2:main',
        ],
    },
)
