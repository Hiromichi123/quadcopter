from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['vision_py']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],

    install_requires=[
        'setuptools',
        'rclpy',
    ],

    zip_safe=True,
    maintainer='Hiromichi123',
    maintainer_email='2271612727@qq.com',
    description='vision package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = scripts.vision_node:main'  # 确保入口路径正确
        ],
    },
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]), #package.xml必须要被安装到install/share/your_package/下
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")), # 所有scripts目录下py文件都被安装
    ],
)