from setuptools import setup

setup(
    name='examplecode-kukarsi',
    version='0.0.1',
    author='Torstein A. Myhre',
    author_email='torstein.a.myhre@ntnu.no',
    description='Control a Kuka robot over RSI',
    long_description='',
    package_dir={'examplecode_kukarsi' : '.',
                 'examplecode_kukarsi.kinematics' : './kinematics',
                 'examplecode_kukarsi.rsi' : './rsi'},
    packages=['examplecode_kukarsi.kinematics',
              'examplecode_kukarsi.rsi'],
    zip_safe=False
)
