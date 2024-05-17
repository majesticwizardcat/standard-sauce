cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(sauce)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" AND USE_NEON)
	message (STATUS "Enabling Neon intrinsics")
	add_compile_definitions("ARM_NEON")
endif()

if (${CMAKE_BUILD_TYPE} MATCHES "Release")
	message(STATUS "Buidling Release")
	add_compile_definitions("NDEBUG")
	add_compile_options(-O3)
else()
	message(STATUS "Buidling Debug")
endif()

include_directories(./)

add_subdirectory(tests)

