from setuptools import find_packages
from setuptools import setup

package_name = 'neato_botvac'

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
    author='Emerson Knapp',
    author_email='emerson.b.knapp@gmail.com',
    maintainer='Emerson Knapp',
    maintainer_email='emerson.b.knapp@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'ROS2 mobile base I/O for a Neato Botvac.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = neato_botvac.test_node:main',
            'neato = neato_botvac.neato_node:main',
        ],
    },
)
