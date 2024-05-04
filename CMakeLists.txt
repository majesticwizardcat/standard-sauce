cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD_REQUIRED true)

add_compile_options(-std=c++20)

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

