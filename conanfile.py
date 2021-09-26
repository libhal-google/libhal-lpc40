from conans import ConanFile


class liblpc40xx_conan(ConanFile):
    name = "liblpc40xx"
    version = "0.0.1"
    license = "Apache License Version 2.0"
    author = "Khalil Estell"
    url = "https://github.com/SJSU-Dev2/liblpc40xx"
    description = "Drivers for the lpc40xx series of microcontrollers using libembeddedhal's abstractions."
    topics = ("peripherals", "hardware")
    exports_sources = "CMakeLists.txt", "liblpc40xx/*"
    no_copy_source = True

    def package(self):
        self.copy("*.hpp")

    def package_id(self):
        self.info.header_only()

    def package_info(self):
        self.cpp_info.includedirs = ["."]

    def requirements(self):
        self.requires("libembeddedhal/0.0.1@")
        self.requires("libxbitset/0.0.1@")
        self.requires("libarmcortex/0.0.1@")
        self.requires("ring-span-lite/0.6.0")
