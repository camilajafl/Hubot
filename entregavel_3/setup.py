from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'entregavel_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # registro do package para ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # pasta de imagens dos olhos
        (
            os.path.join('share', package_name, 'olhos_redimensionados'),
            glob(os.path.join(package_name, 'olhos_redimensionados', '*'))
        ),
        ('share/' + package_name, [os.path.join(package_name, 'encodings.pickle')]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='borg',
    maintainer_email='cmvlara1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'limpador = entregavel_3.limpador:main',
            'olhos_node = entregavel_3.olhos_node:main',
            'ai= entregavel_3.ai:main',
            'facial = entregavel_3.FacialNode:main',
        ],
    },
)
