
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    name='agrodrone',
    package_dir={"agrodrone": "src"},
    packages=["src"]
)

setup(**setup_args) #, requires=['six', 'transitions'])
