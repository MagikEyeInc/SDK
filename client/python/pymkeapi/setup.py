import os
import glob
from setuptools import Extension, setup, Distribution

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "pymkeapi",
    version = "@PYMKEAPI_VERSION@",
    author = "Magik Eye Inc.",
    author_email = "jan@magik-eye.com",
    description = ("Mke API client implementation"),
    license = "BSD-3-Clause",
    license_files = "LICENSE.txt",
    keywords = "pymkeapi client mkeapi",
    url = "http://magik-eye.com",
    packages=[
        'pymkeapi',
        'pymkeapi.open3d',
        'pymkeapi.pcl'
    ],
    python_requires=">=3.6",
    install_requires=[
        'Twisted>=20.3.0',
        'imageio>=2.8.0',
        'numpy>=1.13.3',
        'pyserial>=3.4',
        'psutil>=5.8.0',
        'ssdpy>=0.4.1'
    ],
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Environment :: Console",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: BSD License",
    ]
)
