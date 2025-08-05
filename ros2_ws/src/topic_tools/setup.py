from setuptools import find_packages, setup

package_name = 'topic_tools'

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
    maintainer='robuff',
    maintainer_email='rooks5254@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel2ap = topic_tools.cmd_vel2ap:main',
            'roi_publisher = topic_tools.roi_publisher:main',
            'video_publisher = topic_tools.video_publisher:main',
            'gimbal_publisher = topic_tools.gimbal_publisher:main',
            'img_decoder = topic_tools.img_decoder:main',
        ],
    },
)
