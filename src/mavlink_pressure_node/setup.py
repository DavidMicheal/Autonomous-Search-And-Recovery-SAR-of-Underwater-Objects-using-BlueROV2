# from setuptools import find_packages, setup

# package_name = 'mavlink_pressure_node'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='shubh',
#     maintainer_email='shubhsinghal02@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )


from setuptools import setup

package_name = 'mavlink_pressure_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A custom MAVLink node for SCALED_PRESSURE2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_node = mavlink_pressure_node.pressure_node:main'
        ],
    },
)

# from setuptools import setup

# package_name = 'mavlink_pressure_node'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=[
#         'setuptools',  # Required for ROS 2 Python packages
#         'pymavlink',   # MAVLink library for Python
#     ],
#     zip_safe=True,
#     maintainer='your_name',
#     maintainer_email='your_email@example.com',
#     description='Custom package for MAVLink pressure reading',
#     license='Apache License 2.0',
#     extras_require={
#         'test': ['pytest'],
#     },
#     entry_points={
#         'console_scripts': [
#             'mavlink_pressure_node = my_custom_package.mavlink_pressure_node:main',
#         ],
#     },
# )

