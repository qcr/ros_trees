from setuptools import setup

package_name = 'ros_trees'
package_version = '1.1.0' # Should match package.xml

setup(
    name=package_name,
    version=package_version,
    packages=[
        package_name,
        package_name + '.leaves_common'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Talbo, Gavin Suddrey',
    maintainer_email='b.talbot@qut.edu.au, g.suddrey@qut.edu.au',
    url='https://github.com/qcr/ros_trees',
    description='The ros_trees package',
    license='BSD',
    tests_require=[],
    entry_points={
        'console_scripts': [ ],
    },
)
