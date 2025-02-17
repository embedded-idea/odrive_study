sudo apt-get install gdb-multiarch
sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb


unlock-stlink:
openocd -s /usr/share/openocd/scripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c 'reset halt' -c 'stm32f4x unlock 0'


erase-stlink:
openocd -s /usr/share/openocd/scripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c 'reset halt' -c 'flash erase_sector 0 0 last' -c exit

flash-stlink:
openocd -s /usr/share/openocd/scripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c init  -c 'reset halt' -c 'flash write_image erase build/odrive_min.elf' -c 'reset run' -c exit


gdb:
arm-none-eabi-gdb --ex 'target extended-remote | openocd -s /usrbuild/odrive_min.elf/share/openocd/scripts -f "interface/stlink.cfg" -f "target/stm32f4x.cfg" -c "gdb_port pipe; log_output openocd.log"' --ex 'monitor reset halt' *.ELF


interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/interfaces_template.j2 --output autogen/interfaces.hpp
interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/function_stubs_template.j2 --output autogen/function_stubs.hpp
interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints '..root_interface..' --template fibre-cpp/endpoints_template.j2 --output autogen/endpoints.hpp
interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/type_info_template.j2 --output autogen/type_info.hpp



odrive page


1-MotorControl/main.cpp has main() function as a start point.
please dont read the main() function inside of Board/v3/Src/main.c.

2-Board/v3/board.cpp has system_init(), it will be called by main() as first
step, 
HAL_Init() -  from ST HAL layer
SystemClock_Config() from Board/v3/Src/main.c - set up Osc and Clk for the whole system 
check_board_version() check version , this is from Makefile -D define

3-MotorControl/nvm_config.hpp at this point, need to read cofigurations from 
NVM flash of stm32 , so this hpp class help do it .load and store etc.

config_manager.start_load() && config_read_all() && config_manager.finish_load(&config_size)
&& config_apply_all();

4- Board/v3/board.cpp has board_init()
    MX_GPIO_Init(); Board/v3/Src/gpio.c from STM32CUBEMX
it enables ABCDH gpio group clocks, also set up the following pins.
but my question is what about other pins? todo

M0_nCS_Pin 13 (GPIO_MODE_OUTPUT_PP|GPIO_NOPULL|GPIO_SPEED_FREQ_LOW) on GPIOC
M1_nCS_Pin 14 (GPIO_MODE_OUTPUT_PP|GPIO_NOPULL|GPIO_SPEED_FREQ_LOW) on GPIOC

M1_ENC_Z_Pin 15 (GPIO_MODE_INPUT|GPIO_NOPULL) on GPIOC
GPIO_5_Pin   4 (GPIO_MODE_INPUT|GPIO_NOPULL) on GPIOC
M0_ENC_Z_Pin 9 (GPIO_MODE_INPUT|GPIO_NOPULL) on GPIOC

EN_GATE_Pin 12 (GPIO_MODE_OUTPUT_PP|GPIO_NOPULL|GPIO_SPEED_FREQ_LOW) on GPIOB
nFAULT_Pin 2 (GPIO_MODE_INPUT|GPIO_PULLUP) on GPIOD



    MX_DMA_Init(); DMA setting need more detail later todo

    MX_ADC1_Init(); 10k + 1k , voltage on 1k, use channel 6 on ADC1,which is pin PA6 VBUS_S

    MX_ADC2_Init(); channel 13 regular on ADC2 which is PC3(M1_SO1),channel 10 injection which is PC0(M0_SO1)
get 2 current samples of 3 phases motor.

    MX_TIM1_Init(); looks like it is for motors[0]

    MX_TIM8_Init(); looks like it is for motors[1]

    MX_TIM3_Init(); looks like it is for encoders[0]

    MX_TIM4_Init(); looks like it is for encoders[1]

    MX_SPI3_Init(); PC10 PC11 PC12 communicate with drv8031

    MX_ADC3_Init();channel 12 regular on ADC2 which is PC2(M1_SO2),channel 11 injection which is PC1(M0_SO2)
get 2 current samples of 3 phases motor.

    MX_TIM2_Init(); PB.10 PB.11 output

    MX_TIM5_Init(); looks like PA2 PA3 input capture

    MX_TIM13_Init(); looks like related to motors[0] and motors[1]

    plus interrupts setting

    GPIO_3(PA2) GPIO_4(PA3) could be UART4 if (odrv.config_.enable_uart_a) MX_UART4_Init(); 
also could be if (odrv.config_.enable_uart_b) MX_USART2_UART_Init();

    if enable i2c 
    MX_I2C1_Init(i2c_stats_.addr); the following gpio decide address
    {GPIOA, GPIO_PIN_2}, // GPIO3
    {GPIOA, GPIO_PIN_3}, // GPIO4
    {GPIOC, GPIO_PIN_4}, // GPIO5
    its almost impossible to support i2c , cause pins , if so ,need to take pin away from 
    encoder ABZ and CAN L H, impossible.
    

5- Board/v3/board.cpp
its time to init all GPIOs 
    {nullptr, 0}, // dummy GPIO0 so that PCB labels and software numbers match
    {GPIOA, GPIO_PIN_0}, // GPIO1
    {GPIOA, GPIO_PIN_1}, // GPIO2
    {GPIOA, GPIO_PIN_2}, // GPIO3
    {GPIOA, GPIO_PIN_3}, // GPIO4
    {GPIOC, GPIO_PIN_4}, // GPIO5
    {GPIOB, GPIO_PIN_2}, // GPIO6
    {GPIOA, GPIO_PIN_15}, // GPIO7
    {GPIOB, GPIO_PIN_3}, // GPIO8
    
    {GPIOB, GPIO_PIN_4}, // ENC0_A
    {GPIOB, GPIO_PIN_5}, // ENC0_B
    {GPIOC, GPIO_PIN_9}, // ENC0_Z
    {GPIOB, GPIO_PIN_6}, // ENC1_A
    {GPIOB, GPIO_PIN_7}, // ENC1_B
    {GPIOC, GPIO_PIN_15}, // ENC1_Z
    {GPIOB, GPIO_PIN_8}, // CAN_R
    {GPIOB, GPIO_PIN_9}, // CAN_D

6- MotorControl/main.cpp
rtos_main

MX_USB_DEVICE_Init(); init usb slave device as CDC class

start_general_purpose_adc(); todo

init_communication(); 
    start_uart_server create thread respond data
    start_usb_server create thread respond data


    todo
    pwm0_input.init();

    // Set up the CS pins for absolute encoders (TODO: move to GPIO init switch statement)
    for(auto& axis : axes){
        if(axis.encoder_.config_.mode & Encoder::MODE_FLAG_ABS){
            axis.encoder_.abs_spi_cs_pin_init();
        }
    }

    // Try to initialized gate drivers for fault-free startup.
    // If this does not succeed, a fault will be raised and the idle loop will
    // periodically attempt to reinit the gate driver.
    for(auto& axis: axes){
        axis.motor_.setup();
    }

    for(auto& axis: axes){
        axis.encoder_.setup();
    }

    for(auto& axis: axes){
        axis.acim_estimator_.idq_src_.connect_to(&axis.motor_.Idq_setpoint_);
    }

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();
    start_analog_thread();

    // Wait for up to 2s for motor to become ready to allow for error-free
    // startup. This delay gives the current sensor calibration time to
    // converge. If the DRV chip is unpowered, the motor will not become ready
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i) {
        bool motors_ready = std::all_of(axes.begin(), axes.end(), [](auto& axis) {
            return axis.motor_.current_meas_.has_value();
        });
        if (motors_ready) {
            break;
        }
        osDelay(1);
    }

    for (auto& axis: axes) {
        axis.sensorless_estimator_.error_ &= ~SensorlessEstimator::ERROR_UNKNOWN_CURRENT_MEASUREMENT;
    }

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        axes[i].start_thread();
    }

    odrv.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal).
    vTaskDelete(defaultTaskHandle);





    










odrive video class plan

1- overall introduction of odrive
what are the areas does odrive apply to - robot motor control (FOC)
where to get the hardware and open source code (0.5.6)
what other accessories that need to be purchased to development(stlinkv2,motor,encouder,powersupply)
special introduction of stm32f4x chip


2- first try on odrive board to make it your brusless motor spin
list of items i have 
odrive board 
motor 
encoder 
power supply
micro usb cable
odrivetool  

3- development enviroment setup(hareware and software)
vscode
vscodeplugins
openocd



4- source code learning of ADC data
what is ADC ? How does it work in general
what are the input data odrive needs?
what are the pins connected 
M1_TEMP     PA4
AUX_TEMP    PA5
VBUS_S      PA6
M0_SO1      PC0
M0_SO2      PC1
M1_SO2      PC2
M1_SO1      PC3
M0_TEMP     PC5


ADC1 regular group (VBUS_S) / injection group(VBUS_S)
ADC2 regular group (M1_SO1) / injection group(M0_SO1)
ADC3 regular group (M1_SO2) / injection group(M0_SO2)

TIM1 will triger ADC1 ADC2 ADC3 injection group, means sampling VBUS_S , M0_SO1 M0_SO2
TIM8 will triger ADC2 ADC3 regular group, M1_SO1 M1_SO2
ADC1 regular group will be triggered by ADC_SOFTWARE_START

TIM8_UP_TIM13_IRQHandler will be called when TIM8 expires.this function 
will do 

5- what are the timers
STM32F4 has total is 14 timers as following
TIM1 TIM8 are advanced timers
TIM2-TIM5 TIM9-TIM14 are general timers
TIM6-TIM7 are basic timers

TIM1 168/84M todo ? m0 UVW
TIM2 84M AUX_L(PB10) AUX_H (PB11) output
TIM3 84M encoder0
TIM4 84M encoder1
TIM5 84M GPIO_3(PA2) GPIO_4(PA3) capture
TIM6 not used
TIM7 not used
TIM8 168/84M todo ? m1 UVW
TIM9 not used
TIM10 not used
TIM11 not used
TIM12 not used 
TIM13 84M (kickoff TIM1 and TIM8,measure how long it will take)
TIM14 84M is used for HAL_CLOCK

6- TIM8_UP_TIM13_IRQHandler
There is a flow as following

TIM8_UP_TIM13_IRQHandler->
odrv.sampling_cb();-> will do a loop
axis.encoder_.sample_now();->






setting 
odrv0.erase_configuration()
odrv0.vbus_voltage
odrv0.config.dc_bus_undervoltage_trip_level = 2
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.encoder.config.cpr = 4000
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.05
odrv0.axis0.controller.config.vel_integrator_gain = 0.5
odrv0.axis0.controller.config.enable_overspeed_error = False

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION //长响一声
dump_errors(odrv0)
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH //反转一圈，轻轻响一下
dump_errors(odrv0)
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION //反转，正转一圈
dump_errors(odrv0)
odrv0.axis0.encoder.config.pre_calibrated = True

odrv0.axis0.config.startup_encoder_index_search = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()

odrv0.axis0.controller.input_pos = 5

odrv0.clear_errors()




    {nullptr, 0}, // dummy GPIO0 so that PCB labels and software numbers match

    {GPIOA, GPIO_PIN_0}, // GPIO1       mode is ODriveIntf::GPIO_MODE_UART_A
    {GPIOA, GPIO_PIN_1}, // GPIO2       mode is ODriveIntf::GPIO_MODE_UART_A
    {GPIOA, GPIO_PIN_2}, // GPIO3       mode is ODriveIntf::GPIO_MODE_ANALOG_IN
    {GPIOA, GPIO_PIN_3}, // GPIO4       mode is ODriveIntf::GPIO_MODE_ANALOG_IN
    {GPIOC, GPIO_PIN_4}, // GPIO5       mode is ODriveIntf::GPIO_MODE_ANALOG_IN
    {GPIOB, GPIO_PIN_2}, // GPIO6       mode is ODriveIntf::GPIO_MODE_DIGITAL
    {GPIOA, GPIO_PIN_15}, // GPIO7      mode is ODriveIntf::GPIO_MODE_DIGITAL
    {GPIOB, GPIO_PIN_3}, // GPIO8       mode is ODriveIntf::GPIO_MODE_DIGITAL
    
    {GPIOB, GPIO_PIN_4}, // ENC0_A      mode is ODriveIntf::GPIO_MODE_ENC0
    {GPIOB, GPIO_PIN_5}, // ENC0_B      mode is ODriveIntf::GPIO_MODE_ENC0
    {GPIOC, GPIO_PIN_9}, // ENC0_Z      mode is ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN
    {GPIOB, GPIO_PIN_6}, // ENC1_A      mode is ODriveIntf::GPIO_MODE_ENC1
    {GPIOB, GPIO_PIN_7}, // ENC1_B      mode is ODriveIntf::GPIO_MODE_ENC1
    {GPIOC, GPIO_PIN_15}, // ENC1_Z     mode is ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN
    {GPIOB, GPIO_PIN_8}, // CAN_R       mode is ODriveIntf::GPIO_MODE_CAN_A
    {GPIOB, GPIO_PIN_9}, // CAN_D       mode is ODriveIntf::GPIO_MODE_CAN_A