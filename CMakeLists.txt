# AirPlay 2 Receiver for ESP32
cmake_minimum_required(VERSION 3.16)

# Read firmware version from version.txt
file(READ "${CMAKE_SOURCE_DIR}/version.txt" PROJECT_VER)
string(STRIP "${PROJECT_VER}" PROJECT_VER)

# Add components directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(airplay2-receiver VERSION ${PROJECT_VER})

# Build a SPIFFS image from the data/ directory and flash it alongside the
# firmware. The partition label "storage" matches partitions.csv.
# After adding this, use: idf.py build flash  (flashes both firmware + SPIFFS)
# Or flash SPIFFS only:   idf.py spiffs-flash storage
spiffs_create_partition_image(storage data FLASH_IN_PROJECT)
