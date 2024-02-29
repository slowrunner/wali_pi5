from setuptools import setup

package_name = 'wali'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/ir_dist.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slowrunner',
    maintainer_email='slowrunner@users.noreply.github.com',
    description='WaLI: Wall follower Looking for Intelligence',
    license='If it works for you, let me know',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wali_node = wali.wali_node:main',
            'test_wali_node = wali.test_wali_node:main',
            'vel_pwr_test = wali.vel_pwr_test:main',
            'battery_sub = wali.battery_sub:main',
            'ir2scan = wali.ir2scan:main',
            'ir2scan1 = wali.ir2scan1:main',
            'odometer = wali.odometer:main',
            'say_node = wali.say_node:main',
            'wander = wali.wander:main',
            'wallfollower = wali.wallfollower:main',
            'sub_tf = wali.sub_tf:main',
            'c3interface = wali.c3interface:main',
            'sub_ir = wali.sub_ir:main'
        ],
    },
)
