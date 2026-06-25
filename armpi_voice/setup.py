from setuptools import find_packages, setup

package_name = 'armpi_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tri Hieu',
    maintainer_email='leannewong11@gmail.com',
    description='Voice-commanded control for the Hiwonder ArmPi Ultra.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # command name = module path : function
            'voice_arm_control = armpi_voice.voice_arm_control:main',
            'arm_agent = armpi_voice.arm_agent:main',
        ],
    },
)
