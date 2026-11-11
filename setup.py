from setuptools import setup

package_name = 'armchaos'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    python_requires='>=3.9',
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='darshmenon',
    maintainer_email='darshmenon02@gmail.com',
    description='Break a ros2_control arm on purpose (frozen, late, noisy, NaN or spoofed joint states, a deactivated controller) and restore it.',
    url='https://github.com/darshmenon/armchaos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['armchaos = armchaos.cli:main']},
)
