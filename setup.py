from setuptools import setup, find_packages

package_name = 'auto_nav'

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
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2scanner = auto_nav.r2scanner:main',
            'maze = auto_nav.maze:main',
            'costmap = auto_nav.costmap:main',
            'jervin_rpicam = auto_nav.jervin_rpicam:main',
            'statictf = auto_nav.statictf:main',
        ],
    },
)
