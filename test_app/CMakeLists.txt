# Test runner for i2s_mic.
#
# Pulls in the component (../) and its Unity tests (../test), then runs every
# TEST_CASE on boot. Like the examples, this expects the component folder to
# be named i2s_mic.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ../
    ../test
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(i2s_mic_test_app)
