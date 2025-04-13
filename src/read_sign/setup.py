from setuptools import find_packages, setup

package_name = 'read_sign'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'joblib', 'opencv-python', 'rclpy', 'sensor_msgs', 'scikit-learn'],
    zip_safe=True,
    maintainer='jblevins32',
    maintainer_email='jacob.blevins@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_sign_pubsub = read_sign.read_sign:main'
        ],
    },
)
