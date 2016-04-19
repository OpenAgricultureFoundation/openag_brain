from setuptools import setup, find_packages
from codecs import open

here = path.abspath(path.dirname(__file__))o

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='openag-client-core',
    version='0.0.1',
    description='Core code for running a client of the OpenAg system',
    long_description=long_description,
    url='https://github.com/OpenAgInitiative/openag-client-core',
    author='Open Agriculture Initiative',
    author_email='mitopenag@gmail.com',
    license='GPL',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Environment :: Console',
        'Framework :: Flask',
        'Intended Audience :: Developers,',
        'License :: OSI Approved :: GNU General Public License v2 (GPLv2)',
        'Natural Language :: English',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3.5',
    ],
    packages=[find_packages()],
    namespace_packages['openag', 'openag.client', 'openag.client.modules'],
)
