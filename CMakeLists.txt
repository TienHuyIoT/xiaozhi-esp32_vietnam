# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "2.0.5.06")

# set(CMAKE_CXX_STANDARD 17)

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

# Fix Windows CreateProcess command line length limit (32767 chars).
# WITH many managed components, the -I include list overflows the limit.
# Forcing response files keeps the actual command short: `ccache g++ @rsp`.
if(WIN32)
    set(CMAKE_NINJA_FORCE_RESPONSE_FILE ON CACHE BOOL "Force rsp files" FORCE)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xiaozhi_vn)
