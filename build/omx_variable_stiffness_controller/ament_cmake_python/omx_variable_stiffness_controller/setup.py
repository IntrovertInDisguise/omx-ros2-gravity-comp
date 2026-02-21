from setuptools import find_packages
from setuptools import setup

setup(
    name='omx_variable_stiffness_controller',
    version='0.0.1',
    packages=find_packages(
        include=('omx_variable_stiffness_controller', 'omx_variable_stiffness_controller.*')),
)
