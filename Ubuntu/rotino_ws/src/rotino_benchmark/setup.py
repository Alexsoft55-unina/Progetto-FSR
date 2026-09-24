from setuptools import setup

package_name = 'rotino_benchmark'

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
    maintainer='Alessando Prisco',
    maintainer_email='enzogpt55@libero.it',
    description='RoTino control-law benchmark: campaign runner, logger and comparison',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'logger = rotino_benchmark.logger:main',
            'campaign = rotino_benchmark.campaign:main',
            'compare = rotino_benchmark.compare:main',
            'plot = rotino_benchmark.plot:main',
        ],
    },
)
