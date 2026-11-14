from setuptools import setup

package_name = 'ego_ardupilot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Simulation-only tools for the drone\'s onboard stack (ego_ardupilot) flying '
                'ardupilot_sim: the waypoint survey.',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'survey = ego_ardupilot_sim.survey:main',
        ],
    },
)
