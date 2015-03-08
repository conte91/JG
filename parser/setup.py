#!/usr/bin/env python

from distutils.core import setup
from distutils.extension import Extension

print "building"
setup(name="ApcRobot",
    ext_modules=[
        Extension("apcRobot", ["ipc.cpp"],
        libraries = ["boost_python"])
    ])
