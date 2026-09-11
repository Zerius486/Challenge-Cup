include(CMakeParseArguments)

get_filename_component(STANDALONE_REPO_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

if(NOT CMAKE_CROSSCOMPILING)
  message(FATAL_ERROR
      "Standalone tests require the Arm toolchain; use build_standalone_tests.ps1")
endif()

set(STANDALONE_MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

set(STANDALONE_HAL_SOURCES
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c)

set(STANDALONE_LWIP_SOURCES
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/altcp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/altcp_alloc.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/altcp_tcp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/def.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/dns.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/inet_chksum.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/init.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ip.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/mem.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/memp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/netif.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/pbuf.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/raw.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/stats.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/sys.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/tcp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/tcp_in.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/tcp_out.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/timeouts.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/udp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c
    ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/netif/ethernet.c)

function(add_standalone_firmware)
  set(options NETWORK)
  set(one_value_args NAME)
  set(multi_value_args SOURCES)
  cmake_parse_arguments(TEST "${options}" "${one_value_args}"
                        "${multi_value_args}" ${ARGN})
  if(NOT TEST_NAME OR NOT TEST_SOURCES)
    message(FATAL_ERROR "add_standalone_firmware requires NAME and SOURCES")
  endif()

  set(common_sources
      ${STANDALONE_REPO_ROOT}/Core/Startup/startup_stm32f407xx.s
      ${STANDALONE_REPO_ROOT}/Core/Src/board.c
      ${STANDALONE_REPO_ROOT}/Core/Src/stm32f4xx_it.c
      ${STANDALONE_REPO_ROOT}/Core/Src/syscalls.c
      ${STANDALONE_REPO_ROOT}/Core/Src/sysmem.c
      ${STANDALONE_REPO_ROOT}/Core/Src/system_stm32f4xx.c
      ${STANDALONE_REPO_ROOT}/Projects/ModuleTests/common/Src/standalone_runtime.c
      ${STANDALONE_REPO_ROOT}/Projects/ModuleTests/common/Src/standalone_hooks.c)
  set(vendor_sources ${STANDALONE_HAL_SOURCES})
  if(TEST_NETWORK)
    list(APPEND vendor_sources
        ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
        ${STANDALONE_REPO_ROOT}/Drivers/BSP/Components/lan8742/lan8742.c
        ${STANDALONE_LWIP_SOURCES})
    list(APPEND common_sources
        ${STANDALONE_REPO_ROOT}/Core/Src/ethernetif.c)
  endif()

  set(target "${TEST_NAME}.elf")
  add_executable(${target} ${common_sources} ${TEST_SOURCES} ${vendor_sources})
  target_compile_definitions(${target} PRIVATE STM32F407xx USE_HAL_DRIVER)
  target_include_directories(${target} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/Inc
      ${STANDALONE_REPO_ROOT}/Projects/ModuleTests/common/Inc
      ${STANDALONE_REPO_ROOT}/App/Inc
      ${STANDALONE_REPO_ROOT}/Core/Inc
      ${STANDALONE_REPO_ROOT}/Drivers/CMSIS/Include
      ${STANDALONE_REPO_ROOT}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
      ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc
      ${STANDALONE_REPO_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
      ${STANDALONE_REPO_ROOT}/Drivers/BSP/Components/lan8742
      ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/src/include
      ${STANDALONE_REPO_ROOT}/Middlewares/Third_Party/LwIP/system)
  target_compile_options(${target} PRIVATE
      ${STANDALONE_MCU_FLAGS}
      -ffunction-sections
      -fdata-sections
      -fno-common
      -Wall
      -Wextra
      -Wpedantic
      $<$<CONFIG:Debug>:-Og;-g3>
      $<$<NOT:$<CONFIG:Debug>>:-Os>)
  set_source_files_properties(${common_sources} ${TEST_SOURCES}
      PROPERTIES COMPILE_OPTIONS -Werror)
  set_source_files_properties(${vendor_sources}
      PROPERTIES COMPILE_OPTIONS -Wno-unused-parameter)
  target_link_libraries(${target} PRIVATE m)
  target_link_options(${target} PRIVATE
      ${STANDALONE_MCU_FLAGS}
      -T${STANDALONE_REPO_ROOT}/STM32F407ZGTx_FLASH.ld
      --specs=nano.specs
      --specs=nosys.specs
      -Wl,--gc-sections
      -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}.map
      -Wl,--print-memory-usage)
  set_target_properties(${target} PROPERTIES SUFFIX "")

  add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}>
              ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}.hex
      COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}>
              ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}.bin
      COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
      COMMENT "Creating ${TEST_NAME} HEX/BIN images")
endfunction()
