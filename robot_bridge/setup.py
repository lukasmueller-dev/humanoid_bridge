from setuptools import setup, find_packages

package_name = 'robot_bridge_py'

setup(
 name=package_name,
 version='0.1.0',
 packages=find_packages(),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Xuanhao Song, Puze Liu',
 maintainer_email='puze@robot-learning.de',
 description='Robot bridge package',
 license='MIT',
 tests_require=['pytest'],
)