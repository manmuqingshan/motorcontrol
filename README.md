# motorcontrol
Written specifically for these motor controllers
https://github.com/bgkatz/3phase_integrated
but intended to be easy to port.
 
To build from the command line, install the GNU Arm Embedded Toolchain (arm-none-eabi-gcc) and make, then:

    make            # outputs build/motorcontrol.elf/.bin/.hex
    make DEBUG=1    # -Og/-g3 debug build
    make flash      # flash over ST-Link (STM32_Programmer_CLI or st-flash)
    make clean

Also still builds with ST's Cube IDE:
https://www.st.com/en/development-tools/stm32cubeide.html

Most hardware configuration can be done through hw_config.h
