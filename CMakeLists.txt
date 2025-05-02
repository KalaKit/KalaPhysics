cmake_minimum_required(VERSION 3.30.3)

# Ensure install path is set correctly
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/out/install")
endif()

# Specify the compiler for Linux builds
if (UNIX)
    set(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_MAJOR_VERSION 1)
set(PROJECT_MINOR_VERSION 0)
set(PROJECT_PATCH_VERSION 0)
set(PROJECT_VERSION "${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION}")

project("KalaPhysics" VERSION ${PROJECT_VERSION})

# Set runtime library type for MSVC
if (MSVC)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug")
    else()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")
    endif()
endif()

# Paths to frequently used directories
get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" DIRECTORY)
set(PARENT_DIR_PATH "${PARENT_DIR}")

# Project files
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")

# External files
set(EXT_GLM_DIR "${CMAKE_SOURCE_DIR}/_external_shared/GLM")

# Add source files
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS
    ${SRC_DIR}/*/*.cpp
)

# Shared library
add_library(KalaPhysics SHARED ${SOURCE_FILES})
set_target_properties(KalaPhysics PROPERTIES OUTPUT_NAME "KalaPhysics$<$<CONFIG:Debug>:D>")
target_compile_features(KalaPhysics PRIVATE cxx_std_20)

# Static library
if (NOT WIN32)
	add_library(KalaPhysicsStatic STATIC  ${SOURCE_FILES})
	set_target_properties(KalaPhysicsStatic PROPERTIES OUTPUT_NAME "KalaPhysics$<$<CONFIG:Debug>:D>")
	target_compile_features(KalaPhysicsStatic PRIVATE cxx_std_20)
endif()

# Apply all directories
function(set_kalaphysics_includes target)
	target_include_directories(${target} PRIVATE
        # Library files
        ${INCLUDE_DIR}

        # External libraries
        ${EXT_GLM_DIR}
	)
endfunction()

set_kalaphysics_includes(KalaPhysics)
if (NOT WIN32)
	set_kalaphysics_includes(KalaPhysicsStatic)
endif()

function(configure_kalaphysics_target target)
	if (WIN32)
        target_link_libraries(${target} PRIVATE)
    else()
        target_link_libraries(${target} PRIVATE)
    endif()
endfunction()

configure_kalaphysics_target(KalaPhysics)
if (NOT WIN32)
	configure_kalaphysics_target(KalaPhysicsStatic)
endif()

# Define preprocessor directives
function(set_debug_defines target)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(${target} PRIVATE
            KALAPHYSICS_DLL_EXPORT
            KALAPHYSICS_DEBUG=1
        )
    else()
        target_compile_definitions(${target} PRIVATE
            KALAPHYSICS_DLL_EXPORT
            KALAPHYSICS_DEBUG=0
        )
    endif()
endfunction()

set_debug_defines(KalaPhysics)
if (NOT WIN32)
	set_debug_defines(KalaPhysicsStatic)
endif()

# Copy shared library to bin (.dll or .so)
add_custom_command(TARGET KalaPhysics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/bin"

    # Copy the shared library to the bin directory
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:KalaPhysics>
        "${CMAKE_BINARY_DIR}/bin"

    # Remove the original
    COMMAND ${CMAKE_COMMAND} -E remove
        $<TARGET_FILE:KalaPhysics>
)

# Copy library file to lib folder (.lib or .so)
if (WIN32)
	add_custom_command(TARGET KalaPhysics POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/lib"

		# Copy the shared library to the lib directory
		COMMAND ${CMAKE_COMMAND} -E copy
			$<TARGET_LINKER_FILE:KalaPhysics>
			"${CMAKE_BINARY_DIR}/lib"
	)
else()
	add_custom_command(TARGET KalaPhysicsStatic POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/lib"

		# Copy the static library to the lib directory
		COMMAND ${CMAKE_COMMAND} -E copy
			$<TARGET_LINKER_FILE:KalaPhysicsStatic>
			"${CMAKE_BINARY_DIR}/lib"
	)
endif()

# Copy all headers to include folder
add_custom_command(TARGET KalaPhysics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/include"

    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${INCLUDE_DIR}/"
        "${CMAKE_BINARY_DIR}/include"
)