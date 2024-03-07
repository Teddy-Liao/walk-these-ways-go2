from setuptools import find_packages
from distutils.core import setup

setup(
    name='go2_gym',
    version='1.0.0',
    author='Dengting Liao',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='540614105@qq.com',
    description='Toolkit for deployment of sim-to-real RL on the Unitree Go2.',
    install_requires=['ml_logger==0.8.117',
                      'ml_dash==0.3.20',
                      'jaynes>=0.9.2',
                      'params-proto==2.10.5',
                      'gym>=0.14.0',
                      'tqdm',
                      'matplotlib',
                      'numpy==1.23.5'
                      ]
)
