from setuptools import setup

package_name = 'cg_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rnicola',
    maintainer_email='rodrigo.nicola0@gmail.com',
    description='Teleop for Culling Games',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'teleop_keyboard = cg_teleop.teleop_node:main',
        ],
    },
)
