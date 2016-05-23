from os import path
import sys
from codecs import open
from setuptools import setup, find_packages

if sys.version_info < (3, 5):
    raise RuntimeError("openag_brain requires Python 3.5 or newer")

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='openag_brain',
    version='0.0.1',
    description='Framework for software modules in an OpenAg food computer',
    long_description=long_description,
    url='https://github.com/OpenAgInitiative/openag_brain',
    author='Open Agriculture Initiative',
    author_email='mitopenag@gmail.com',
    license='GPL',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Environment :: Console',
        'Framework :: Flask',
        'Intended Audience :: Developers,',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3.5',
    ],
    packages=find_packages(),
    namespace_packages=[
        'openag', 'openag.brain', 'openag.brain.modules',
        'openag.brain.module_groups'
    ],
    include_package_data=True,
    install_requires=[
        'couchdb>=1.0.1',
        'flask>=0.10.1',
        'gevent>=1.1.1',
    ],
    entry_points={
        'console_scripts': [
            'openag_init_db = openag.brain.core.init_db:main',
            'openag_run_modules = openag.brain.core.run_modules:main'
        ]
    }
)
