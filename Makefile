nRF52_SDK		= 	nrf52sdk_14.0.0
S132_HEX_NAME   =   s132_nrf52_5.0.0_softdevice.hex
S132_HEX 		= 	$(nRF52_SDK)/components/softdevice/s132/hex/$(S132_HEX_NAME)

include $(nRF52_SDK)/sdk.mk

NO_ECHO := @

#CC = ~/Tools/GNU_ARM_GCC/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-gcc
#SIZE = ~/Tools/GNU_ARM_GCC/gcc-arm-none-eabi-5_4-2016q3/bin/arm-none-eabi-size

CC = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OPTIMIZATION = -O0
BUILD_FOLDER = Build_Output
OUTPUT_BINARY_FOLDER = $(BUILD_FOLDER)
OUTPUT_BINARY_NAME = CarTracker
LINKER_SCRIPT = armgcc_s132_nrf52832_xxaa.ld
LINKER_COMMON_SCRIPT = nrf5x_common.ld

#------------------------ INCLUDE PATHS --------------------------------

INC_PATHS += -Iinc/
INC_PATHS += -Ihardware/
INC_PATHS += -Ibluetooth/
INC_PATHS += -Ibluetooth/services/
INC_PATHS += -Iutils/
INC_PATHS += -I$(nRF52_SDK)/components/ble/common/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/util/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/fifo/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/experimental_section_vars/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/fstorage/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/log/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/strerror/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/experimental_log/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/experimental_log/src/
INC_PATHS += -I$(nRF52_SDK)/components/libraries/experimental_memobj/
INC_PATHS += -I$(nRF52_SDK)/components/softdevice/common/
INC_PATHS += -I$(nRF52_SDK)/config/
INC_PATHS += $(SDK_INCLUDE_PATHS) 

#----------------------- PROJECT SOURCES ------------------------------

ASM_SOURCE_FILES += gcc_startup_nrf52.s
ASM_SOURCE_FILES += aes_cortexm.s

C_SOURCE_FILES = src/main.c
C_SOURCE_FILES += src/driving_analysis.c
C_SOURCE_FILES += src/system_nrf52.c
C_SOURCE_FILES += src/fifo.c
C_SOURCE_FILES += src/request_fifos.c
C_SOURCE_FILES += src/tasks.c
C_SOURCE_FILES += utils/crypto.c
C_SOURCE_FILES += utils/external_flash_driver.c
C_SOURCE_FILES += utils/file_system.c
C_SOURCE_FILES += utils/gps.c
C_SOURCE_FILES += utils/gsm.c
C_SOURCE_FILES += utils/lsm6dsm.c
C_SOURCE_FILES += utils/parsing_utils.c
C_SOURCE_FILES += utils/scheduler.c
#C_SOURCE_FILES += utils/nfc.c
C_SOURCE_FILES += hardware/Systick.c
C_SOURCE_FILES += hardware/UART.c
C_SOURCE_FILES += hardware/SPI.c 
C_SOURCE_FILES += hardware/RTC.c
C_SOURCE_FILES += hardware/GPIOTE.c
C_SOURCE_FILES += hardware/internal_flash.c
C_SOURCE_FILES += bluetooth/ble_common.c
C_SOURCE_FILES += bluetooth/advertising.c
C_SOURCE_FILES += bluetooth/ble_central.c
C_SOURCE_FILES += bluetooth/services/ble_uart_service.c
C_SOURCE_FILES += bluetooth/services/ble_uart_service_central.c
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/common/ble_advdata.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/util/app_error.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/util/app_error_weak.c
C_SOURCE_FILES += $(nRF52_SDK)/components/softdevice/common/nrf_sdh.c
C_SOURCE_FILES += $(nRF52_SDK)/components/softdevice/common/nrf_sdh_ble.c
C_SOURCE_FILES += $(nRF52_SDK)/components/softdevice/common/nrf_sdh_soc.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/experimental_section_vars/nrf_section_iter.c
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/ble_db_discovery/ble_db_discovery.c
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c 
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/common/ble_conn_state.c
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/common/ble_srv_common.c
#C_SOURCE_FILES += $(nRF52_SDK)/components/ble/common/ble_conn_params.c
C_SOURCE_FILES += $(nRF52_SDK)/components/ble/ble_advertising/ble_advertising.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/fstorage/nrf_fstorage.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/util/sdk_mapped_flags.c
C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/fifo/app_fifo.c
C_SOURCE_FILES += src/cmsis_dsp/cmsis_dsp_math.c
#----------------------- COMPILING FLAGS ------------------------------

CFLAGS +=	$(OPTIMIZATION)
CFLAGS += 	-mcpu=cortex-m4
CFLAGS +=   -mabi=aapcs
CFLAGS +=   -mthumb
CFLAGS += 	-mfloat-abi=hard
CFLAGS += 	-mfpu=fpv4-sp-d16
CFLAGS +=   -DARM_MATH_CM4
CFLAGS +=	--std=gnu99
CFLAGS += 	-Werror
CFLAGS +=	-fmessage-length=0
CFLAGS +=	-ffunction-sections
CFLAGS +=	-fdata-sections
CFLAGS += 	-fno-strict-aliasing
CFLAGS +=	-g3
CFLAGS += 	-fno-builtin --short-enums
#CFLAGS += 	-fomit-frame-pointer
CFLAGS +=	-DNRF52832_XXAA
CFLAGS += 	-DNRF52832
CFLAGS +=   -DNRF52
CFLAGS +=   -DBLE_STACK_SUPPORT_REQD
CFLAGS +=   -DDEBUG
CFLAGS +=   -DNRF_FSTORAGE_ENABLED
CFLAGS +=   -DNRF_BLE_CONN_PARAMS_ENABLED
CFLAGS +=   -DNRF_SD_BLE_API_VERSION=4
CFLAGS +=   -DNRF_SDH_ENABLED=1
CFLAGS +=   -DNRF_SDH_BLE_ENABLED=1
CFLAGS +=   -DNRF_SDH_SOC_ENABLED=1
CFLAGS +=   -DNRF_SECTION_ITER_ENABLED=1
CFLAGS +=   -DNRF_BLE_GATT_ENABLED=1
CFLAGS +=   -DBLE_DB_DISCOVERY_ENABLED=1
CFLAGS +=   -DBLE_ADVERTISING_ENABLED=1
CFLAGS +=   -DAPP_FIFO_ENABLED=1
CFLAGS +=   -DS132
CFLAGS +=   -DNRF_SDH_BLE_PERIPHERAL_LINK_COUNT=1
CFLAGS +=   -DNRF_SDH_BLE_CENTRAL_LINK_COUNT=1
CFLAGS +=   -DECB=1										 #AES cipher method 
CFLAGS +=   -D__FPU_PRESENT=1
CFLAGS +=   -DARM_MATH_CM4=1

ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DARM_MATH_CM4
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb
ASMFLAGS += -mfloat-abi=hard
ASMFLAGS += -mfpu=fpv4-sp-d16
ASMFLAGS += -D__STARTUP_CLEAR_BSS	# Needed to zero BSS section in RAM by the startup code
ASMFLAGS += -D__FPU_PRESENT=1
#------------------------ LINKER FLAGS --------------------------------

LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mabi=aapcs
LDFLAGS += -mthumb
LDFLAGS += -mfloat-abi=hard
LDFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS += --specs=nano.specs -lc -lnosys -lgcc -lm
LDFLAGS += -Xlinker -Map=$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -T"$(nRF52_SDK)/$(LINKER_SCRIPT)"
LDFLAGS += -L"$(nRF52_SDK)"
#LDFLAGS += -L/home/konrad/Projects/Bare_Metal/SecuCar/Car_Tracker/libs/fpu


#C_SOURCE_FILES += $(nRF52_SDK)/components/libraries/experimental_log/src/nrf_log_frontend.c
                 
#------------------------ COMPILATION VARIABLES ------------------------

# List of source file names without directories
C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))

# List of paths where the source files lie
C_PATHS = $(dir $(C_SOURCE_FILES))

# List of object files prefixed with build folder where the objects file will be held
C_OBJECTS = $(addprefix $(BUILD_FOLDER)/, $(C_SOURCE_FILE_NAMES:.c=.o))

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))

ASM_PATHS = $(dir $(ASM_SOURCE_FILES))

ASM_OBJECTS = $(addprefix $(BUILD_FOLDER)/, $(ASM_SOURCE_FILE_NAMES:.s=.o))

# List of paths where the sources are to be searched for
vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

#------------------------------- TARGETS -------------------------------

# Working target to compile a single file - called for each of the .c file
$(BUILD_FOLDER)/%.o: %.c
	@echo "Compiling file: $<"
	$(CC) $(CFLAGS) $(INC_PATHS) -c $< -o $@
	
# Assemble files
$(BUILD_FOLDER)/%.o: %.s
	@echo Compiling ASM file: $(notdir $<)
	$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Working target for linking compiled files
$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf: $(OBJECTS)
	$(info info: ------- GENERATING ELF FILE -------)
	@echo "Linking files..."
	$(CC) $(LDFLAGS) $(OBJECTS)  -o $@
	arm-none-eabi-objcopy -O ihex "$@" "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).hex"
	arm-none-eabi-objcopy -O binary "$@" "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).bin"
	arm-none-eabi-objdump -S "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf" > $(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).lss
	@echo "Finished linking..."

	
# Target for main compilation
all: print_start_info $(BUILD_FOLDER) $(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf
	@echo "Finished building: $@"
	@echo " "
	@echo "Total hex size:"
	$(NO_ECHO)$(SIZE) "$(OUTPUT_BINARY_FOLDER)/$(OUTPUT_BINARY_NAME).elf"

# Target for cleaning the directory
clean:
	rm -rf $(BUILD_FOLDER)
	@echo "Build directory cleaned."
	
# Target for logging compilation start
print_start_info:
	$(info info: -------- COMPILING FILES ---------)
	@echo "Starting building with optimization: $(OPTIMIZATION)"
	
# Target for creating build folder
$(BUILD_FOLDER):
	@mkdir $(BUILD_FOLDER)
	
program_softdevice:
	nrfjprog -e
	nrfjprog --program $(S132_HEX)
	# Disable NFC pins for MAIN BOARD
	nrfjprog --memwr 0x1000120C --val 0xFFFFFFFE 	
