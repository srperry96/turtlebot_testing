from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['turn_north'],
    package_dir={'turn_north': 'src/turn_north'}
)
setup(**d)
