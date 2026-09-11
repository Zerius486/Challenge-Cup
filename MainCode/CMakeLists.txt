cmake_minimum_required(VERSION 3.22)

project(ChallengeCup_Main VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(MSVC)
  set(PROJECT_WARNING_OPTIONS /W4 /WX)
else()
  set(PROJECT_WARNING_OPTIONS -Wall -Wextra -Wpedantic -Werror)
endif()

set(MATH_LIBRARY)
if(CMAKE_CROSSCOMPILING OR UNIX)
  set(MATH_LIBRARY m)
endif()

set(PROTOCOL_SOURCES
    App/Src/arm_controller.c
    App/Src/arm_kinematics.c
    App/Src/arm_nuc_protocol.c
    App/Src/erob_canopen.c
    App/Src/force_sensor.c
    App/Src/legacy_can_bridge.c
    App/Src/legacy_udp.c
    App/Src/robstride.c
    App/Src/udp_protocol.c)

add_library(protocol_core STATIC ${PROTOCOL_SOURCES})
target_include_directories(protocol_core PUBLIC App/Inc)
target_compile_options(protocol_core PRIVATE ${PROJECT_WARNING_OPTIONS})

if(CMAKE_CROSSCOMPILING)
  enable_language(ASM)
  set(MCU_FLAGS
      -mcpu=cortex-m4
      -mthumb
      -mfpu=fpv4-sp-d16
      -mfloat-abi=hard)
  target_compile_options(protocol_core PRIVATE ${MCU_FLAGS})

  set(HAL_SOURCES
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c)

  set(LWIP_SOURCES
      Middlewares/Third_Party/LwIP/src/core/altcp.c
      Middlewares/Third_Party/LwIP/src/core/altcp_alloc.c
      Middlewares/Third_Party/LwIP/src/core/altcp_tcp.c
      Middlewares/Third_Party/LwIP/src/core/def.c
      Middlewares/Third_Party/LwIP/src/core/dns.c
      Middlewares/Third_Party/LwIP/src/core/inet_chksum.c
      Middlewares/Third_Party/LwIP/src/core/init.c
      Middlewares/Third_Party/LwIP/src/core/ip.c
      Middlewares/Third_Party/LwIP/src/core/mem.c
      Middlewares/Third_Party/LwIP/src/core/memp.c
      Middlewares/Third_Party/LwIP/src/core/netif.c
      Middlewares/Third_Party/LwIP/src/core/pbuf.c
      Middlewares/Third_Party/LwIP/src/core/raw.c
      Middlewares/Third_Party/LwIP/src/core/stats.c
      Middlewares/Third_Party/LwIP/src/core/sys.c
      Middlewares/Third_Party/LwIP/src/core/tcp.c
      Middlewares/Third_Party/LwIP/src/core/tcp_in.c
      Middlewares/Third_Party/LwIP/src/core/tcp_out.c
      Middlewares/Third_Party/LwIP/src/core/timeouts.c
      Middlewares/Third_Party/LwIP/src/core/udp.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c
      Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c
      Middlewares/Third_Party/LwIP/src/netif/ethernet.c)

  set(APPLICATION_SOURCES
      Core/Src/board.c
      Core/Src/ethernetif.c
      Core/Src/main.c
      Core/Src/stm32f4xx_it.c
      Core/Src/syscalls.c
      Core/Src/sysmem.c
      Core/Src/system_stm32f4xx.c
      App/Src/app_bridge.c
      App/Src/vofa_firewater.c
      App/Src/udp_transport.c)

  set_source_files_properties(${APPLICATION_SOURCES}
      PROPERTIES COMPILE_OPTIONS -Werror)
  set_source_files_properties(${HAL_SOURCES}
      Drivers/BSP/Components/lan8742/lan8742.c
      PROPERTIES COMPILE_OPTIONS -Wno-unused-parameter)

  add_executable(ChallengeCup_Main.elf
      Core/Startup/startup_stm32f407xx.s
      ${APPLICATION_SOURCES}
      Drivers/BSP/Components/lan8742/lan8742.c
      ${HAL_SOURCES}
      ${LWIP_SOURCES})

  target_link_libraries(ChallengeCup_Main.elf PRIVATE protocol_core ${MATH_LIBRARY})
  target_compile_definitions(ChallengeCup_Main.elf PRIVATE
      STM32F407xx
      USE_HAL_DRIVER)
  target_include_directories(ChallengeCup_Main.elf PRIVATE
      App/Inc
      Core/Inc
      Drivers/CMSIS/Include
      Drivers/CMSIS/Device/ST/STM32F4xx/Include
      Drivers/STM32F4xx_HAL_Driver/Inc
      Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
      Drivers/BSP/Components/lan8742
      Middlewares/Third_Party/LwIP/src/include
      Middlewares/Third_Party/LwIP/system)
  target_compile_options(ChallengeCup_Main.elf PRIVATE
      ${MCU_FLAGS}
      -ffunction-sections
      -fdata-sections
      -fno-common
      -Wall
      -Wextra
      -Wpedantic
      $<$<CONFIG:Debug>:-Og;-g3>
      $<$<NOT:$<CONFIG:Debug>>:-Os>)
  target_link_options(ChallengeCup_Main.elf PRIVATE
      ${MCU_FLAGS}
      -T${CMAKE_SOURCE_DIR}/STM32F407ZGTx_FLASH.ld
      --specs=nano.specs
      --specs=nosys.specs
      -Wl,--gc-sections
      -Wl,-Map=${CMAKE_BINARY_DIR}/ChallengeCup_Main.map
      -Wl,--print-memory-usage)
  set_target_properties(ChallengeCup_Main.elf PROPERTIES SUFFIX "")

  add_custom_command(TARGET ChallengeCup_Main.elf POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:ChallengeCup_Main.elf>
              ${CMAKE_BINARY_DIR}/ChallengeCup_Main.hex
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:ChallengeCup_Main.elf>
              ${CMAKE_BINARY_DIR}/ChallengeCup_Main.bin
      COMMAND ${CMAKE_SIZE} $<TARGET_FILE:ChallengeCup_Main.elf>
      COMMENT "Creating HEX/BIN images and reporting image size")
else()
  enable_testing()
  add_executable(protocol_tests Tests/test_protocols.c)
  target_link_libraries(protocol_tests PRIVATE protocol_core ${MATH_LIBRARY})
  target_compile_options(protocol_tests PRIVATE ${PROJECT_WARNING_OPTIONS})
  add_test(NAME protocol_tests COMMAND protocol_tests)

  add_executable(app_bridge_tests
      Tests/test_app_bridge.c
      Tests/mocks/board.c
      App/Src/app_bridge.c)
  target_include_directories(app_bridge_tests PRIVATE Tests/mocks App/Inc)
  target_compile_definitions(app_bridge_tests PRIVATE
      APP_REMOTE_MOTION_ALLOWED=1)
  target_compile_options(app_bridge_tests PRIVATE ${PROJECT_WARNING_OPTIONS})
  target_link_libraries(app_bridge_tests PRIVATE protocol_core ${MATH_LIBRARY})
  add_test(NAME app_bridge_tests COMMAND app_bridge_tests)
endif()
