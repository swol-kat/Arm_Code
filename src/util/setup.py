from setuptools import setup

setup(
    name='util',
    version='0.0',
    description='Just a whole buncha useful shit',
    author='Arjun gandhi',
    author_email='yeet',
    packages=['src/util'],  # same as name
    install_requires=['matplotlib', 'numpy'],  # external packages as dependencies
)