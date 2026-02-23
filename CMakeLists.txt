cmake_minimum_required(VERSION 3.29.2)

set(PROGRAM_VERSION "KalaPhysics")
set(PROGRAM_VERSION_NUMBER 1.0.0.0)

project("KalaPhysics" VERSION ${PROGRAM_VERSION_NUMBER} LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Runtime lib handling for MSVC
if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Platform Detection
if (WIN32)
    message(STATUS "[KALAPHYSICS] Platform = Windows")
elseif(UNIX)
	message(STATUS "[KALAPHYSICS] Platform = Unix")
else()
    message(FATAL_ERROR "[KALAPHYSICS] Unsupported platform. Only Windows and Unix are supported.")
endif()

# Build Type Detection
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(IS_DEBUG TRUE)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(IS_RELEASE TRUE)
else()
    message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: '${CMAKE_BUILD_TYPE}'! Must be Debug, Release, or RelWithDebInfo.")
endif()

# Directory Setup
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(EXT_SHARED_DIR "${CMAKE_SOURCE_DIR}/_external_shared")

# Source Files
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*/*.cpp"
)

# SHARED LIBRARY (.dll)
add_library(KalaPhysics SHARED ${SOURCE_FILES})

# Enable exception handling for MSVC
if (MSVC)
    target_compile_options(KalaPhysics PRIVATE /EHsc)
endif()
	
# Get suffix
if (IS_RELEASE)
	set(KALA_SUFFIX "")
else()
	set(KALA_SUFFIX "d")
endif()

# Set output name with renderer suffix and debug postfix
set_target_properties(KalaPhysics PROPERTIES
	OUTPUT_NAME "KalaPhysics${PROJECT_VERSION_MAJOR}${KALA_SUFFIX}"
)

# Includes
file(GLOB_RECURSE HEADERS
	configure_depends
	"${CMAKE_SOURCE_DIR}/include/*.hpp"
)
target_sources(KalaPhysics PRIVATE ${HEADERS})
target_include_directories(KalaPhysics PRIVATE
	"${INCLUDE_DIR}"
	"${EXT_SHARED_DIR}"
)

if (WIN32)
	# Preprocessor Defines
	target_compile_definitions(KalaPhysics PRIVATE 
		LIB_EXPORT          # dll import/export
		WIN32_LEAN_AND_MEAN # cleaner windows.h
		NOMINMAX            # skips min/max windows macros in favor of the std variants
		UNICODE             # selects wide api variants over ansi for UTF16 windows functions
		_UNICODE            # uses wide text types in the C runtime layer
	)
endif()
	
# Expose version string
add_compile_definitions(
	PROGRAM_VERSION="${PROGRAM_VERSION}"
)

# Hide console in release mode
#if(IS_RELEASE)
#    set_target_properties(KalaPhysics PROPERTIES WIN32_EXECUTABLE TRUE)
#    if (MSVC)
#        set_target_properties(KalaPhysics PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
#    endif()
#endif()

# Copy important files
set(COPY_TARGETS README.md LICENSE.md)
foreach(FILE ${COPY_TARGETS})
add_custom_command(TARGET KalaPhysics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E rm -f 
        "$<TARGET_FILE_DIR:KalaPhysics>/${FILE}"
    COMMAND ${CMAKE_COMMAND} -E copy
		"${CMAKE_SOURCE_DIR}/${FILE}"
		"$<TARGET_FILE_DIR:KalaPhysics>/${FILE}"
)
endforeach()

# Copy entire docs folder
add_custom_command(TARGET KalaPhysics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E rm -rf
        "$<TARGET_FILE_DIR:KalaPhysics>/docs"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/docs"
        "$<TARGET_FILE_DIR:KalaPhysics>/docs"
)
