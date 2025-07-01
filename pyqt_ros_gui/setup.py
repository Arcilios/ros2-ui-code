from setuptools import setup, find_packages

package_name = 'pyqt_ros_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        'pyqt_ros_gui.actions': ['actions.json'],  # 关键点：让 actions.json 被安装
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui_joint_gui.launch.py']),
        ('share/pyqt_ros_gui/actions', ['pyqt_ros_gui/actions/actions.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='PyQt5 GUI for controlling joints via ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gui_node = pyqt_ros_gui.main:main',
        'joint_node_receive = pyqt_ros_gui.node.joint_node_receive:main',
        'joint_node_publish = pyqt_ros_gui.node.joint_node_publish:main',
    ]
},

)
