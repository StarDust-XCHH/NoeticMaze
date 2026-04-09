cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME NoeticMaze)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)


# Hardware FPU Configuration (STM32F446 - Cortex-M4)
add_compile_options(
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
)
add_link_options(
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -Wl,-u,_printf_float
        -Wl,-u,_scanf_float
)

# Create an executable object type
# 递归查找 App 目录下所有的 .c 和 .h 文件
# CONFIGURE_DEPENDS 的作用是：当你在这些目录下新增或删除文件时，CMake 会自动重新配置，无需手动重载。
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
        "App/*.c"
        "App/*.h"
)

# 使用收集到的文件列表生成可执行文件
add_executable(${CMAKE_PROJECT_NAME} ${APP_SOURCES})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)


set(DSP_SOURCE_DIR "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source")
file(GLOB DSP_SOURCES
        "${DSP_SOURCE_DIR}/CommonTables/arm_common_tables.c"
        "${DSP_SOURCE_DIR}/FastMathFunctions/arm_cos_f32.c"
        "${DSP_SOURCE_DIR}/FastMathFunctions/arm_sin_f32.c"
        "${DSP_SOURCE_DIR}/FastMathFunctions/arm_atan2_f32.c"
        "${DSP_SOURCE_DIR}/BasicMathFunctions/arm_offset_f32.c"
        "${DSP_SOURCE_DIR}/SupportFunctions/arm_copy_q15.c"
        "${DSP_SOURCE_DIR}/ControllerFunctions/arm_pid_init_f32.c"
)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${DSP_SOURCES}
)

# Add include paths
# 1. 自动查找 App 目录下所有的头文件，并提取它们所在的目录
file(GLOB_RECURSE ALL_APP_HEADERS "${CMAKE_SOURCE_DIR}/App/*.h")
set(APP_INCLUDE_DIRS "")
foreach(header_file ${ALL_APP_HEADERS})
    get_filename_component(header_dir ${header_file} DIRECTORY)
    list(APPEND APP_INCLUDE_DIRS ${header_dir})
endforeach()
# 2. 去除重复的目录路径
if(APP_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES APP_INCLUDE_DIRS)
endif()
# 3. 添加到目标包含路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # 自动生成的 App 子目录路径
        ${APP_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/App/lidar/Inc
        # 固定的 CMSIS/库 路径（这些通常不需要递归，保持原样即可）
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        USE_HAL_DRIVER
        STM32F446xx
        ARM_MATH_CM4
        EMPL_TARGET_STM32F4
        MPU6500
)

# Link options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 定义生成 hex 和 bin 的命令
set(HEX_FILE ${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${BIN_FILE}
        COMMENT "Generating .hex and .bin files from .elf..."
)
