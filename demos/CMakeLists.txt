# NOTE: Output pin demo is used here just for testing. Find better location for
# this.
add_executable(output_pin_demo output_pin.cpp)

target_link_libraries(output_pin_demo
                      PRIVATE
                      liblpc40xx
                      libembeddedhal::libembeddedhal
                      libxbitset::libxbitset
                      libarmcortex::libarmcortex)
