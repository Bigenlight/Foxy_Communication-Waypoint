from setuptools import setup

package_name = 'mqtt_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='MQTT subscriber node for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_subscriber = mqtt_subscriber.mqtt_subscriber:main',
        ],
    },
)
