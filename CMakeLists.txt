cmake_minimum_required(VERSION 3.22)


# Use the ARM GCC toolchain by default, including non-preset builds.
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
    CACHE FILEPATH "ARM GCC toolchain file")

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME base_control)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-Wl,-u,_printf_float"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Collect user sources, including files added after configuration.
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.S"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    ${CMAKE_CURRENT_SOURCE_DIR}/User/App/Interface
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Bsp/Interface
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Modules/Interface
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Devices/Interface
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Utils/Interface
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MiddleWares/FreeRTOS")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)