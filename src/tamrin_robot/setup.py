from setuptools import find_packages, setup
from glob import glob

package_name = 'tamrin_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/description', glob('description/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/setups', glob('setups/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saquib',
    maintainer_email='saquibvswild@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

