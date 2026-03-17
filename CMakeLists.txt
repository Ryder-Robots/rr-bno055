# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.16)
project(rr_bno055
  VERSION 0.0.0
  DESCRIPTION "Ryder Robots BNO055 IMU hardware driver"
  LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ─── FetchContent: Bosch BNO055 SensorAPI ────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
  bno055_sensorapi
  GIT_REPOSITORY https://github.com/boschsensortec/BNO055_SensorAPI.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)

# Populate but do not add_subdirectory — it has no CMakeLists.txt
FetchContent_Populate(bno055_sensorapi)

# bno055_sensorapi_SOURCE_DIR is now set by FetchContent
set(BNO055_API_DIR ${bno055_sensorapi_SOURCE_DIR})

# ─── Library target ──────────────────────────────────────────────────────────
add_library(rr_bno055
  src/bno055_device.cpp
  src/hardware_transport.cpp
  src/i2c_hardware_transport.cpp
  src/uart_hardware_transport.cpp
  src/transport_factory.cpp
  ${BNO055_API_DIR}/bno055.c        # vendored Bosch C source
)

target_include_directories(rr_bno055
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${BNO055_API_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/bno055_sensorapi>
)

target_compile_options(rr_bno055 PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

# ─── Install rules ───────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS rr_bno055
  EXPORT rr_bno055Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install Bosch headers alongside rr_bno055 headers
install(FILES
  ${BNO055_API_DIR}/bno055.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bno055_sensorapi
)

# Install Bosch LICENSE for legal compliance
install(FILES
  ${BNO055_API_DIR}/LICENSE
  DESTINATION ${CMAKE_INSTALL_DATADIR}/rr_bno055/vendor/bno055_sensorapi
  RENAME LICENSE.bno055_sensorapi
)

install(EXPORT rr_bno055Targets
  FILE rr_bno055Targets.cmake
  NAMESPACE rr::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rr_bno055
)

# ─── CMake package config ────────────────────────────────────────────────────
include(CMakePackageConfigHelpers)

configure_package_config_file(
  cmake/rr_bno055Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rr_bno055Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rr_bno055
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rr_bno055ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rr_bno055Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rr_bno055ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rr_bno055
)

# ─── Unit tests (GTest) ──────────────────────────────────────────────────────
include(CTest)
enable_testing()

find_package(GTest REQUIRED)

add_executable(rr_bno055_tests
  test/test_transport_config.cpp
  test/test_bno055_transport_config.cpp
  test/test_transport_factory.cpp
  test/test_hardware_transport.cpp
  test/test_bno055_device.cpp
)

target_link_libraries(rr_bno055_tests
  rr_bno055
  GTest::GTest
  GTest::Main
)

target_compile_options(rr_bno055_tests PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_test(NAME rr_bno055_tests COMMAND rr_bno055_tests)

# ─── Debian package (CPack) ──────────────────────────────────────────────────
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "rr-bno055")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT "Ryder Robots <aaron@ryderrobots.com>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PROJECT_DESCRIPTION})
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

# Debian dev package (headers + cmake config)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
  "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst"
)

include(CPack)