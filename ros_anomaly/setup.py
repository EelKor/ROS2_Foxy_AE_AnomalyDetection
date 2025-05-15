from setuptools import setup

package_name = 'ros_anomaly'

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
    maintainer='wsl',
    maintainer_email='TBD',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rostf_anoamly = ros_anomaly.rostf_anomaly:main',
            'visualization = ros_anomaly.visualization:main',
        ],
    },
)
