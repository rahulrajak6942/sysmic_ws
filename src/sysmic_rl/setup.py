from setuptools import find_packages, setup

package_name = 'sysmic_rl'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh-khandelwal',
    maintainer_email='shubh4664@gmail.com',
    description='RL package for dual-arm control on the Sysmic bot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sysmic_rl = sysmic_rl:main',
            'sysmic_rl_train = sysmic_rl.train:main',
            'sysmic_rl_test = sysmic_rl.test:main',
            'sysmic_rl_run = sysmic_rl.run:main',
        ],
    },
)
