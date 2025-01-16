from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_rov'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),        
        (os.path.join('share', package_name, 'config'), glob('config/*.config.yaml')),
        ('share/ament_index/resource_index/packages',
        ['resource/autonomous_rov']),
        ('share/autonomous_rov', ['package.xml']),
        ('share/autonomous_rov/launch', ['launch/mavros_launch.py']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg'))
    ],
    install_requires=['setuptools','opencv-python-headless','cv_bridge'],
    zip_safe=True,
    maintainer='vincent',
    maintainer_email='vincent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'RedBoxDetector = autonomous_rov.RedBoxDetector:main',
        'image_extractor = autonomous_rov.image_extractor:main',
        'red_box_detection = autonomous_rov.Blackbox_detection:main',
        'video_object_detection = autonomous_rov.video_object_detection:main',
        'listenerMIR = autonomous_rov.listenerMIR:main',
        'video = autonomous_rov.video:main',
        'manual_controller = autonomous_rov.manual_controller:main',
        'depth_controller = autonomous_rov.depth_controller:main',
        'RobotGrasper = autonomous_rov.RobotGrasper:main',
        'gui_node = autonomous_rov.gui_node:main',
        'pid_gui_node = autonomous_rov.pid_gui_node:main',
        ],       
        
    },
)
data_files=[
    
],
