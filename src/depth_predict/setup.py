from setuptools import find_packages
from setuptools import setup
import os
from glob import glob

### ここにパッケージ名を記述 ###
package_name = 'depth_predict'

### 必要に応じてパッケージ情報を記述 ###
setup(
    name=package_name,
    version='0.7.8',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Hayato Takeyama',
    author_email='takehaya.724@gmail.com',
    maintainer='Hayato Takeyama',
    maintainer_email='takehaya.724@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'pubsubpy sample'
    ),
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = node.Pub:main',
            'sub = node.Sub:main',
            'predict = node.Predict:main',
        ],
    },
)
### ↑ entry_pointsの部分に、パッケージに含まれるノード(pythonソース)を列挙 ###
