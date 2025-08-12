from setuptools import find_packages, setup
import glob

package_name = 'gluon_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob.glob('meshes/**/*', recursive=True)),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*.urdf*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panda',
    maintainer_email='wang.liang@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
