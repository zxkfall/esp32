# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/xingkun.zhang/Documents/tools/esp/esp-idf/components/bootloader/subproject"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/tmp"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/src"
  "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/xingkun.zhang/Documents/study/code/esp32/hello_world/cmake-build-debug/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
