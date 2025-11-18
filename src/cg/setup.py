import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.csv')),
        (os.path.join('share', package_name, 'lobotomy_kaisen'), glob('../../lobotomy_kaisen/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rnicola',
    maintainer_email='rodrigo.nicola0@gmail.com',
    description='Culling Games',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'test = cg.teste:main',
            'maze = cg.main:game',
            'edit = cg.main:editor',
        ],
    },
)
