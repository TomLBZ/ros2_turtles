from setuptools import find_packages, setup

package_name = 'robmove'

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
    maintainer='LBZ',
    maintainer_email='libozhao142857@hotmail.com',
    description='This package contains all the logic for this project: \
  After countdown, controlling two robots to perform different movements \
  either simultaneously or one after the other, with blocking actions; \
  and being able to repeat the movements after a set amount of time.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mov = robmove.mov:main'
        ],
    },
)
