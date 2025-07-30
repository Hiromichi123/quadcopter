from setuptools import setup

package_name = 'yolip'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'torchvision',
        'ultralytics',
        'OpenCV',
        'pyyaml',
        'matplotlib',
        'tqdm',
        'numpy',
        'rclpy',
        'messages', # 消息包
        'cv_bridge', # ROS2图像转换
        'sensor_msgs', # 图像消息
    ],
    zip_safe=True,
    maintainer='Hiromichi123',
    maintainer_email='2271612727@qq.com',
    description='clip + yolo包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolip = yolip.main:main'
        ],
    },
)
