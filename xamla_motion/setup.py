

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name='xamla_motion',
    version='0.0.1',
    author='Manuel Volk',
    author_email='manuel.volk@xamla.com',
    packages=['xamla_motion', 'xamla_motion.types'],
    package_dir={'': 'src'},
    url='https://github.com/Xamla/pythonClientLib_XamlaMotion',
    license='LICENSE.txt',
    description='Clint library to interact with xamla motion lib',
    long_description=open('README.md').read(),
    install_requires=[
        "Numpy >= 1.1.1",
    ],
)

setup(**setup_args)
