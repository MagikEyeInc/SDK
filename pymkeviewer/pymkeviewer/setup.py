import os
import glob
from setuptools import Extension, setup, Distribution

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "pymkeviewer",
    version = "@PYMKEVIEWER_VERSION@",
    author = "Magik Eye Inc.",
    author_email = "jan@magik-eye.com",
    description = ("Mke Point Cloud Viewer"),
    license = "BSD-3-Clause",
    license_files = "LICENSE.txt",
    keywords = "pymkeapi client viewer",
    url = "http://magik-eye.com",
    packages=[
        'pymkeviewer'
    ],
    package_dir={'pymkeviewer': 'pymkeviewer'},
    package_data={'pymkeviewer': ['icon/*.png']},

    python_requires=">=3.6",
    install_requires=[
        'PySide2>=5.15.2',
        'vispy>=0.6.6',
        'pymkeapi==@PYMKEAPI_VERSION@'
    ],
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Environment :: Console",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: BSD License",
    ],
    entry_points={
        "console_scripts": [
            "pymkeviewer = pymkeviewer.pymkeviewer:main_func"
        ]
    }
)
