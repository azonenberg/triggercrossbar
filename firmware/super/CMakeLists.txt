########################################################################################################################
# Top level config

cmake_minimum_required(VERSION 3.19)
project(triggercrossbar-super LANGUAGES C CXX ASM)

file(REAL_PATH "${CMAKE_SOURCE_DIR}/../.." PROJECT_ROOT)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

########################################################################################################################
# Pull in target config

# Common boilerplate we don't want to replicate in every project
include("${PROJECT_ROOT}/common-embedded-platform/boilerplate.cmake")

# Target-specific configuration
set(TARGET_MCU stm32l031)
include("${PROJECT_ROOT}/stm32-cpp/build/${TARGET_MCU}.cmake")

# Include paths
include_directories(
	${CMAKE_SOURCE_DIR}
	${PROJECT_ROOT}
	${PROJECT_ROOT}/etl/include
	)

# keep this until we switch the crossbar to the hardware rev 0.5 IBC
add_compile_definitions(LEGACY_IBC)

########################################################################################################################
# Build all of our other dependencies

# Do not build network service classes
set(CEP_BUILD_SERVICES 0)

add_subdirectory("${PROJECT_ROOT}/common-embedded-platform" "${CMAKE_BINARY_DIR}/common-embedded-platform")
add_subdirectory("${PROJECT_ROOT}/embedded-cli" "${CMAKE_BINARY_DIR}/embedded-cli")
add_subdirectory("${PROJECT_ROOT}/embedded-utils" "${CMAKE_BINARY_DIR}/embedded-utils")
add_subdirectory("${PROJECT_ROOT}/microkvs" "${CMAKE_BINARY_DIR}/microkvs")
add_subdirectory("${PROJECT_ROOT}/stm32-cpp" "${CMAKE_BINARY_DIR}/stm32-cpp")

########################################################################################################################
# The actual project build

add_subdirectory("bsp")
# no bootloader right now
#add_subdirectory("bl")
add_subdirectory("main")
