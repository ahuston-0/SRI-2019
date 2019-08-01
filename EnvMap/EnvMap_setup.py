from distutils.core import setup

from Cython.Build import cythonize

setup(
    name='EnvMap',
    ext_modules=cythonize("*.pyx"),
)
