cmake_minimum_required(VERSION 3.22)
project(stm32g474_line_follow C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(HAL_ROOT Drivers/STM32G4xx_HAL_Driver)
set(DEVICE_ROOT Drivers/CMSIS/Device/ST/STM32G4xx)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS "${HAL_ROOT}/Src/*.c")
add_library(stm32_hal STATIC ${HAL_SOURCES} ${DEVICE_ROOT}/Source/Templates/system_stm32g4xx.c)
target_include_directories(stm32_hal PUBLIC Core/Inc ${HAL_ROOT}/Inc ${HAL_ROOT}/Inc/Legacy ${DEVICE_ROOT}/Include Drivers/CMSIS/Include)
target_compile_definitions(stm32_hal PUBLIC USE_HAL_DRIVER STM32G474xx)
target_compile_options(stm32_hal PUBLIC ${MCU_FLAGS} -ffunction-sections -fdata-sections)
target_compile_options(stm32_hal PRIVATE -Os)
add_executable(line-following.elf Core/Src/main.c Core/Src/syscalls.c Board/board.c Board/lcd_st7735.c
    Control/line_follower.c Control/bridge_logic.c
    ${DEVICE_ROOT}/Source/Templates/gcc/startup_stm32g474xx.s)
target_include_directories(line-following.elf PRIVATE Board Control)
target_link_libraries(line-following.elf PRIVATE stm32_hal m)
target_compile_options(line-following.elf PRIVATE -Wall -Wextra -Wpedantic -Werror
    $<$<CONFIG:Debug>:-Og;-g3> $<$<CONFIG:Release>:-Os>)
target_link_options(line-following.elf PRIVATE ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32G474VETX_FLASH.ld --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/line-following.map -u _printf_float)
set_property(TARGET line-following.elf APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/STM32G474VETX_FLASH.ld)
add_custom_command(TARGET line-following.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:line-following.elf> ${CMAKE_CURRENT_BINARY_DIR}/line-following.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:line-following.elf> ${CMAKE_CURRENT_BINARY_DIR}/line-following.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:line-following.elf>)
