from setuptools import setup

package_name = 'wall_follower_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Wall following + trash detector',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wall_follower = wall_follower_bot.wall_follower:main',
            'trash_detector = wall_follower_bot.trash_detector:main'
        ],
    },
)
