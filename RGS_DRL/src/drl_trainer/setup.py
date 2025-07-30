from setuptools import setup
import glob

package_name = 'drl_trainer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuxuan',
    maintainer_email='zhao2002yx@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'laser_processor = drl_trainer.laser_processor:main',
        	'differential_forwarder = drl_trainer.differential_forwarder:main',
        	'rl_trainer = drl_trainer.trainer:main',
        	'ackerman_forwarder = drl_trainer.ackerman_forwarder:main',
        	'sb3_trainer = drl_trainer.sb3_trainer:main',
        ],
    },
)
