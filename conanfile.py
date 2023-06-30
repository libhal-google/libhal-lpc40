#!/usr/bin/python
#
# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import copy
from conan.tools.build import check_min_cppstd
import os


required_conan_version = ">=1.50.0"


class libhal_lpc40_conan(ConanFile):
    name = "libhal-lpc40"
    version = "2.0.0"
    license = "Apache-2.0"
    url = "https://github.com/conan-io/conan-center-index"
    homepage = "https://libhal.github.io/libhal-lpc40"
    description = ("A collection of drivers and libraries for the LPC40 "
                   "series microcontrollers from NXP")
    topics = ("arm", "microcontroller", "lpc", "lpc40",
              "lpc40xx", "lpc4072", "lpc4074", "lpc4078", "lpc4088")
    settings = "compiler", "build_type", "os", "arch"
    exports_sources = "include/*", "linker_scripts/*", "tests/*", "LICENSE", "CMakeLists.txt", "src/*"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualBuildEnv"
    no_copy_source = True

    options = {
        "platform": [
            "lpc4072",
            "lpc4074",
            "lpc4076",
            "lpc4078",
            "lpc4088",
            "not-me"
        ],
    }
    default_options = {
        "platform": "not-me",
    }

    @property
    def _is_me(self):
        return (self.options.platform == "lpc4078" or
                self.options.platform == "lpc4076" or
                self.options.platform == "lpc4088" or
                self.options.platform == "lpc4074" or
                self.options.platform == "lpc4072")

    @property
    def _min_cppstd(self):
        return "20"

    @property
    def _compilers_minimum_version(self):
        return {
            "gcc": "11",
            "clang": "14",
            "apple-clang": "14.0.0"
        }

    @property
    def _bare_metal(self):
        return self.settings.os == "baremetal"

    def validate(self):
        if self.settings.get_safe("compiler.cppstd"):
            check_min_cppstd(self, self._min_cppstd)

    def build_requirements(self):
        self.tool_requires("libhal-cmake-util/1.0.0")
        self.test_requires("boost-ext-ut/1.1.9")

    def requirements(self):
        self.requires("libhal/[^2.0.0]")
        self.requires("libhal-util/[^2.0.0]")
        self.requires("ring-span-lite/[^0.6.0]")
        self.requires("libhal-armcortex/[^2.0.0]")

    def layout(self):
        cmake_layout(self)

    def build(self):
        run_test = not self.conf.get("tools.build:skip_test", default=False)

        cmake = CMake(self)
        if self.settings.os == "Windows":
            cmake.configure()
        elif self._bare_metal:
            cmake.configure(variables={
                "BUILD_TESTING": "OFF"
            })
        else:
            cmake.configure(variables={"ENABLE_ASAN": True})

        cmake.build()

        if run_test and not self._bare_metal:
            test_folder = os.path.join("tests")
            self.run(os.path.join(test_folder, "unit_test"))

    def package(self):
        copy(self,
             "LICENSE",
             dst=os.path.join(self.package_folder, "licenses"),
             src=self.source_folder)
        copy(self,
             "*.h",
             dst=os.path.join(self.package_folder, "include"),
             src=os.path.join(self.source_folder, "include"))
        copy(self,
             "*.hpp",
             dst=os.path.join(self.package_folder, "include"),
             src=os.path.join(self.source_folder, "include"))
        copy(self,
             "*.ld",
             dst=os.path.join(self.package_folder, "linker_scripts"),
             src=os.path.join(self.source_folder, "linker_scripts"))

        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_target_name", "libhal::lpc40")
        self.cpp_info.libs = ["libhal-lpc40"]

        if self._bare_metal and self._is_me:
            linker_path = os.path.join(self.package_folder, "linker_scripts")
            link_script = "-Tlibhal-lpc40/" + str(self.options.platform) + ".ld"
            self.cpp_info.exelinkflags = ["-L" + linker_path, link_script]
