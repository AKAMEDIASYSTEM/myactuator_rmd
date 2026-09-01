"""
@file setup.py
@mainpage
   Make pip package management system invoke CMake; Adapted from https://github.com/pybind/cmake_example
@author
   Tobit Flatscher (github.com/2b-t)
"""

import os
from pathlib import Path
import subprocess
import sys

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())

class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        cmake_args = [
            f"-D CMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-D Python3_EXECUTABLE={sys.executable}",
            f"-D CMAKE_BUILD_TYPE={cfg}",
            f"-D PYTHON_BINDINGS=on"
        ]

        # CMake cannot find pybind11 on its own when pip builds in an isolated environment, since the package is
        # unpacked somewhere temporary that is on sys.path but not on CMAKE_PREFIX_PATH. Hand it the directory
        # explicitly, the same way one would on the command line with
        # -D pybind11_DIR=$(python3 -m pybind11 --cmakedir).
        try:
            import pybind11
            cmake_args.append(f"-D pybind11_DIR={pybind11.get_cmake_dir()}")
        except ImportError:
            print("warning: pybind11 not importable, leaving CMake to locate it by itself", file=sys.stderr)

        build_args = []
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True
        )
        subprocess.run(
            ["cmake", "--build", ".", *build_args], cwd=build_temp, check=True
        )

setup(
    ext_modules=[CMakeExtension("myactuator_rmd_py")],
    cmdclass={"build_ext": CMakeBuild}
)
