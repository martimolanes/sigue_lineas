from setuptools import setup

package_name = 'driverless'

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
    maintainer='francorbacho',
    maintainer_email='fran.corbachoflores@gmail.com',
    description='Handles autonomous driving logic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driverless = driverless.driverless:main'
        ],
    },
)
