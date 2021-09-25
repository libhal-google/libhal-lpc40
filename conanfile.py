from conans import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake
from conan.tools.layout import cmake_layout


class liblpc40xx_conan(ConanFile):
    name = "liblpc40xx"
    version = "0.0.1"
    license = "Apache License Version 2.0"
    author = "Khalil Estell"
    url = "https://github.com/SJSU-Dev2/liblpc40xx"
    description = "A collection of interfaces and abstractions for embedded peripherals and devices using modern C++"
    topics = ("peripherals", "hardware")
    exports_sources = "CMakeLists.txt", "liblpc40xx/*"
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake_find_package_multi"

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake_debug = CMake(self)
        cmake_debug.configure()
        cmake_debug.build()

    def package(self):
        cmake = CMake(self)
        self.copy("*.hpp")
        self.copy("*.cpp")
        cmake.install()

    def package_info(self):
        self.cpp_info.includedirs = ["."]
        self.cpp_info.libs = ["liblpc40xx"]

    def requirements(self):
        self.requires("libembeddedhal/0.0.1@")
        self.requires("libxbitset/0.0.1@")
        self.requires("libarmcortex/0.0.1@")
