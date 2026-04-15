cmake_minimum_required(VERSION 3.16)

project("data_sensor"
    VERSION "1.0.0"
    DESCRIPTION "Personalized scripts for reading sensor descriptor of 3DM-CV7-AHRS using C"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# TODO: Set MIP_SDK_ROOT_DIR if building as a standalone project
if(DEFINED MIP_SDK_ROOT_DIR OR DEFINED CACHE{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "${MIP_SDK_ROOT_DIR}")
elseif(DEFINED ENV{MIP_SDK_ROOT_DIR})
    set(_MIP_SDK_ROOT_DIR "$ENV{MIP_SDK_ROOT_DIR}")
else()
    set(_MIP_SDK_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# We need to find MicroStrain and MIP SDK in a standalone project
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
    list(APPEND CMAKE_PREFIX_PATH "${_MIP_SDK_ROOT_DIR}")

    find_package(mip REQUIRED CONFIG)

    # This example uses a serial connection and requires the microstrain_serial component
    find_package(microstrain REQUIRED COMPONENTS microstrain_serial CONFIG)
elseif(NOT MICROSTRAIN_ENABLE_SERIAL)
    # microstrain_serial library is needed for source built projects
    message(FATAL_ERROR "The serial connections library is required for this example")
endif()

# Make sure the MIP libraries are found
if(NOT DEFINED CACHE{MIP_LIBRARIES} AND NOT DEFINED MIP_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MIP libraries.")
endif()

# Make sure the MicroStrain libraries are found
if(NOT DEFINED CACHE{MICROSTRAIN_LIBRARIES} AND NOT DEFINED MICROSTRAIN_LIBRARIES)
    message(FATAL_ERROR "Could not find the required MicroStrain libraries.")
endif()

set(SCRIPTS
	scaled_accel.c
	scaled_gyro.c
	scaled_accel2.c
)

foreach(src ${SCRIPTS})
	# Get project name without file extension of set list
	get_filename_component(exec_name ${src} NAME_WE)

	# Create the example
	add_executable(${exec_name} "${CMAKE_CURRENT_LIST_DIR}/${src}")
	
	# Configure the project in a sub-folder for IDEs that support it
	if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_LIST_DIR})
	    file(RELATIVE_PATH PROJECT_RELATIVE_PATH "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_LIST_DIR}/..")
	    set_target_properties(${exec_name} PROPERTIES
        	FOLDER "${PROJECT_RELATIVE_PATH}"
	    )
	endif()

	target_link_libraries(${exec_name} PRIVATE
	    ${MIP_LIBRARIES}
	    ${MICROSTRAIN_LIBRARIES}
	)

	target_include_directories(${exec_name} PRIVATE
	    ${MIP_INCLUDE_DIRS}
	    ${MICROSTRAIN_INCLUDE_DIRS}
	)

	if(MSVC)
	    set(COMPILE_OPTIONS
	        "/W4" # Enable warnings
	        "/WX" # Warnings as errors
	        "/MP" # Multi-process compilation
	    )
	else()
	    set(COMPILE_OPTIONS
        	"-Wall"   # Enable warnings
	        "-Wextra" # Enable extra warnings
	        "-Werror" # Warnings as errors
	    )
	endif()

	target_compile_options(${exec_name} PRIVATE
	    "${COMPILE_OPTIONS}"
	)

	target_compile_definitions(${exec_name} PRIVATE
	    "MICROSTRAIN_LOGGING_MAX_LEVEL=MICROSTRAIN_LOGGING_LEVEL_INFO_" # Info logging level is required at a minimum
	)

endforeach()
