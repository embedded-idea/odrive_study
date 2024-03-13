######################################
# target
######################################
TARGET = odrive_min


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES_HAL_DRIVER = \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

#
C_SOURCES_BOARD = \
ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Drivers/DRV8301/drv8301.cpp \
Board/v3/board.cpp \
Board/v3/Src/main.c \
Board/v3/Src/stm32f4xx_hal_timebase_TIM.c \
Board/v3/Src/tim.c \
Board/v3/Src/dma.c \
Board/v3/Src/freertos.c \
Board/v3/Src/usbd_conf.c \
Board/v3/Src/spi.c \
Board/v3/Src/usart.c \
Board/v3/Src/usbd_cdc_if.c \
Board/v3/Src/adc.c \
Board/v3/Src/stm32f4xx_hal_msp.c \
Board/v3/Src/usbd_desc.c \
Board/v3/Src/stm32f4xx_it.c \
Board/v3/Src/usb_device.c \
Board/v3/Src/can.c \
Board/v3/Src/system_stm32f4xx.c \
Board/v3/Src/gpio.c \
Board/v3/Src/i2c.c

C_SOURCES_RTOS = \
ThirdParty/FreeRTOS/Source/croutine.c \
ThirdParty/FreeRTOS/Source/event_groups.c \
ThirdParty/FreeRTOS/Source/list.c \
ThirdParty/FreeRTOS/Source/queue.c \
ThirdParty/FreeRTOS/Source/stream_buffer.c \
ThirdParty/FreeRTOS/Source/tasks.c \
ThirdParty/FreeRTOS/Source/timers.c \
ThirdParty/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
ThirdParty/FreeRTOS/Source/portable/MemMang/heap_4.c

C_SOURCES_USB = \
ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_core.c \
ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
ThirdParty/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

C_SOURCES_FIRMWARE = \
syscalls.c \
MotorControl/utils.cpp \
MotorControl/arm_sin_f32.c \
MotorControl/arm_cos_f32.c \
MotorControl/low_level.cpp \
MotorControl/axis.cpp \
MotorControl/motor.cpp \
MotorControl/thermistor.cpp \
MotorControl/encoder.cpp \
MotorControl/endstop.cpp \
MotorControl/acim_estimator.cpp \
MotorControl/mechanical_brake.cpp \
MotorControl/controller.cpp \
MotorControl/foc.cpp \
MotorControl/open_loop_controller.cpp \
MotorControl/oscilloscope.cpp \
MotorControl/sensorless_estimator.cpp \
MotorControl/trapTraj.cpp \
MotorControl/pwm_input.cpp \
MotorControl/main.cpp \
Drivers/STM32/stm32_system.cpp \
Drivers/STM32/stm32_gpio.cpp \
Drivers/STM32/stm32_nvm.c \
Drivers/STM32/stm32_spi_arbiter.cpp \
communication/can/can_simple.cpp \
communication/can/odrive_can.cpp \
communication/communication.cpp \
communication/ascii_protocol.cpp \
communication/interface_uart.cpp \
communication/interface_usb.cpp \
communication/interface_i2c.cpp \
FreeRTOS-openocd.c \
autogen/version.c

C_SOURCES_FIBRE = \
fibre-cpp/fibre.cpp \
fibre-cpp/channel_discoverer.cpp \
fibre-cpp/legacy_protocol.cpp


C_SOURCES = $(C_SOURCES_HAL_DRIVER) \
$(C_SOURCES_BOARD) \
$(C_SOURCES_RTOS) \
$(C_SOURCES_USB) \
$(C_SOURCES_FIRMWARE) \
$(C_SOURCES_FIBRE)



# ASM sources
ASM_SOURCES = \
Board/v3/startup_stm32f405xx.s

# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

#try to get a correct python command to run all python scripts for me is python3 -B
ifeq ($(shell python -c "import sys; print(sys.version_info.major)"), 3)
	PY_CMD := python -B
else
	PY_CMD := python3 -B
endif

# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++ -std=c++17 -Wno-register
CC = $(GCC_PATH)/$(PREFIX)gcc -std=c99
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CXX = $(PREFIX)g++ -std=c++17 -Wno-register
CC = $(PREFIX)gcc -std=c99
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 


# fpu
FPU = -DFPU_FPV4
# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F405xx \
-DHW_VERSION_MAJOR=3 \
-DHW_VERSION_MINOR=6 \
-DHW_VERSION_VOLTAGE=56


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES_CMSIS = \
-IThirdParty/CMSIS/Include \
-IThirdParty/CMSIS/Device/ST/STM32F4xx/Include

C_INCLUDES_HAL_Driver = \
-IThirdParty/STM32F4xx_HAL_Driver/Inc


C_INCLUDES_BOARD = \
-IBoard/v3/Inc \
-IThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F

C_INCLUDES_RTOS = \
-IThirdParty/FreeRTOS/Source/include \
-IThirdParty/FreeRTOS/Source/CMSIS_RTOS

C_INCLUDES_USB = \
-IThirdParty/STM32_USB_Device_Library/Core/Inc \
-IThirdParty/STM32_USB_Device_Library/Class/CDC/Inc

C_INCLUDES_FIREWARE = \
-IMotorControl

C_INCLUDES_FIBRE = \
-Ifibre-cpp/include


C_INCLUDES = -I./ $(C_INCLUDES_CMSIS) \
$(C_INCLUDES_HAL_Driver) \
$(C_INCLUDES_BOARD) \
$(C_INCLUDES_RTOS) \
$(C_INCLUDES_USB) \
$(C_INCLUDES_FIREWARE) \
$(C_INCLUDES_FIBRE)




# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += -D__weak="__attribute__((weak))"
CFLAGS += -D__packed="__attribute__((__packed__))"
CFLAGS += -DUSE_HAL_DRIVER
CFLAGS += -mthumb
CFLAGS += -mfloat-abi=hard
CFLAGS += -Wno-psabi
CFLAGS += -Wall -Wdouble-promotion -Wfloat-conversion -fdata-sections -ffunction-sections
CFLAGS += -DFIBRE_ENABLE_SERVER
CFLAGS += -Wno-nonnull
CFLAGS += -DFIBRE_ENABLE_SERVER=1


ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Board/v3/STM32F405RGTx_FLASH.ld

# libraries
LIBS = -flto -lc -lm -lnosys -larm_cortexM4lf_math
LIBDIR = \
-L./ThirdParty/CMSIS/Lib/GCC

LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += -mthumb -mfloat-abi=hard -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections
LDFLAGS += -Wl,--undefined=uxTopUsedPriority

# default action: build all
all: AUTOHEADERS $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin TESTBIN_HEX_ASM

AUTOHEADERS:
	@echo $(PY_CMD)
	@mkdir -p autogen
	@$(PY_CMD) tools/odrive/version.py --output autogen/version.c
	@$(PY_CMD) interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/interfaces_template.j2 --output autogen/interfaces.hpp
	@$(PY_CMD) interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/function_stubs_template.j2 --output autogen/function_stubs.hpp
	@$(PY_CMD) interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints 'ODrive3' --template fibre-cpp/endpoints_template.j2 --output autogen/endpoints.hpp
	@$(PY_CMD) interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/type_info_template.j2 --output autogen/type_info.hpp

#######################################
# build the application
#######################################
# list of objects
OBJECTS_C = $(filter %.c,$(C_SOURCES))
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(OBJECTS_C:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS_CXX = $(filter %.cpp,$(C_SOURCES))
OBJECTS += $(addprefix $(BUILD_DIR)/__,$(notdir $(OBJECTS_CXX:.cpp=.o)))
vpath %.cpp $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "ccccccccccccccccccccccccccccc"
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/__%.o: %.cpp Makefile | $(BUILD_DIR) 
	@echo "c++++++++++++++++++++++++++++"
	$(CXX) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@


$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

TESTBIN_HEX_ASM: $(BUILD_DIR)/$(TARGET).elf
#display the size
	arm-none-eabi-size $(BUILD_DIR)/$(TARGET).elf
#create *.hex and *.bin output formats
	arm-none-eabi-objcopy -O ihex $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex
	arm-none-eabi-objcopy -O binary -S $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	arm-none-eabi-objdump $(BUILD_DIR)/$(TARGET).elf -dSC > $(BUILD_DIR)/$(TARGET).asm
	


#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR) autogen
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
