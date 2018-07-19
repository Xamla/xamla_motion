

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name='xamla_motion',
    version='0.0.1',
    description='client library to interact with xamla motion / rosvita',
    long_description=open('README.md').read(),
    classifiers=[
        'Development Status :: 3 - Alpha',
        'License :: OSI Approved :: GNU General Public License v2 (GPLv2)',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Topic :: Scientific/Engineering',
        ],
    keywords='funniest joke comedy flying circus',
    author='Manuel Volk',
    author_email='manuel.volk@xamla.com',
    packages=['xamla_motion',
              'xamla_motion.xamla_motion_exceptions',
              'xamla_motion.data_types',
              'xamla_motion.examples'],
    package_dir={'': 'src'},
    url='https://github.com/Xamla/pythonClientLib_XamlaMotion',
    license='LICENSE',
    install_requires=[
        'numpy >= 1.1.1',
        'pyquaternion >= 0.9.2',
        'asyncio >= 3.4.3',
        'rospkg >= 1.1.4',
        'catkin_pkg >= 0.4.5',
        'rospy >= 1.12.13',
    ],
)

setup(**setup_args)
