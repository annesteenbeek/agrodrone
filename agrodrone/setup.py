from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    name='agrodrone',
    package_dir={"agrodrone": "src"},
    packages=["src"]
)

setup(**d, requires=['six', 'transitions'])
