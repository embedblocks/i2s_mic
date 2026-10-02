cmake_minimum_required(VERSION 3.16)

# Vendors i2s_mic locally (this example lives inside the i2s_mic
# repository) — i2s_spk is instead resolved via the ESP Component Manager
# from main/idf_component.yml.
set(EXTRA_COMPONENT_DIRS
    ../../
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(i2s_mic_loopback_dual_i2s_example)
