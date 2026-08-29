set(MCU_VARIANT stm32g431xx)
set(JLINK_DEVICE stm32g431cb)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32G431CBUx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32G431xx
    HSE_VALUE=8000000
    )
endfunction()
