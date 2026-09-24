import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pointcloud_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        # URDF/Xacro
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),

        # Worlds
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')
        ),

        # Config
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),

        # Room
        (
            os.path.join('share', package_name, 'models', 'room'),
            glob('models/room/*.sdf') +
            glob('models/room/*.config')
        ),
        (
            os.path.join('share', package_name, 'models', 'room', 'mesh'),
            glob('models/room/mesh/*')
        ),

        # Table
        (
            os.path.join('share', package_name, 'models', 'table'),
            glob('models/table/*.sdf') +
            glob('models/table/*.config')
        ),
        (
            os.path.join('share', package_name, 'models', 'table', 'mesh'),
            glob('models/table/mesh/*')
        ),

        # Chair
        (
            os.path.join('share', package_name, 'models', 'chair'),
            glob('models/chair/*.sdf') +
            glob('models/chair/*.config')
        ),
        (
            os.path.join('share', package_name, 'models', 'chair', 'mesh'),
            glob('models/chair/mesh/*')
        ),

        # Couch
        (
            os.path.join('share', package_name, 'models', 'couch'),
            glob('models/couch/*.sdf') +
            glob('models/couch/*.config')
        ),
        (
            os.path.join('share', package_name, 'models', 'couch', 'mesh'),
            glob('models/couch/mesh/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='sarwan-sadique',
    maintainer_email='sarwansadiq13@todo.com',

    description='ROS 2 3D point cloud simulation robot',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'teleop_node = pointcloud_bot.teleop_node:main',
            'pointcloud_mapper = pointcloud_bot.pointcloud_mapper:main',
        ],
    },
)