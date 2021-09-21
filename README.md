# stm32-vcu
Project based on the OpenInverter System by Johannes Huebner to provide a universal VCU (Vehicle Control Unit) for electric vehicle conversion projects. 

Please visit the development thread on the Openinverter Forum for more information : https://openinverter.org/forum/viewtopic.php?f=3&t=1277


Video on progress : https://vimeo.com/506480876

# Features

- Nissan Leaf Gen1/2/3 inverter via CAN
- Nissan Leaf Gen2 PDM (Charger and DCDC)
- CCS DC fast charge via BMW i3 LIM
- Lexus GS450H inverter / gearbox via sync serial
- Toyota Prius/Yaris/Auris Gen 3 inverters via sync serial
- BMW E46 CAN support
- BMW E39 CAN support
- BMW E65 CAN Support
- Mid 2000s VAG Can support
- Opel Ampera / Chevy Volt 6.5kw cabin heater

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedencies are libopencm3 and libopeninv. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-vcu by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface

I use CodeBlocks IDE :  https://www.codeblocks.org/

Sept 21 : V1 hardware and firmware from "Master" branch now running in my E46 Touring and E39 vehicles.
