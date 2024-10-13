from setuptools import setup

package_name = 'my_joint_state_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A subscriber node to read joint states',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_subscriber = my_joint_state_subscriber.joint_state_subscriber:main',
        ],
    },
)

