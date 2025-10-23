from setuptools import find_packages, setup
from glob import glob

package_name = 'utils'

# Create a mapping of other files to be copied in the src -> install
# build.  This is a list of tuples.  The first entry in the tuple is
# the install folder into which to place things.  The second entry is
# a list of files to place into that folder.
otherfiles = [
    ('share/' + package_name + '/rviz', glob('rviz/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133 Utilities Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plotjoints         = utils.plotjoints:main',
            'plottranslation    = utils.plottranslation:main',
            'plotorientation    = utils.plotorientation:main',
            'plotcondition      = utils.plotcondition:main',
        ],
    },
)
