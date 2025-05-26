# Set Project Configs
cmake_minimum_required(VERSION 3.30)
project(DA_PROJ1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define source and header directories
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src/main_files")
set(HEADERS_DIR "${CMAKE_SOURCE_DIR}/src/headers")
set(DATA_STRUCTURES_DIR "${CMAKE_SOURCE_DIR}/src/data_structures")

# Gather source files
file(GLOB_RECURSE SOURCES "${SRC_DIR}/*.cpp")
file(GLOB_RECURSE HEADERS "${HEADERS_DIR}/*.h" "${DATA_STRUCTURES_DIR}/*.h")

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
include_directories(${HEADERS_DIR} ${DATA_STRUCTURES_DIR})
