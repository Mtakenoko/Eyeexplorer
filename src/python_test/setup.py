from setuptools import find_packages
from setuptools import setup

package_name = 'python_test'

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
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes which were previously in the ros2/examples repository '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = python_programs.listener:main',
            'talker = python_programs.talker:main'
        ],
    },
)
