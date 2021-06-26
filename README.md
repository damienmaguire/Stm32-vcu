# stm32-vcu
Project based on the OpenInverter System by Johannes Huebner to provide a universal VCU (Vehicle Control Unit) for electric vehicle conversion projects. 

Please visit the development thread on the Openinverter Forum for more information : https://openinverter.org/forum/viewtopic.php?f=3&t=1277

NOTE : Not for use in a car as of this commit. Alpha status for bench testing only!

Video on progress : https://vimeo.com/506480876

# Features

-CCS DC Fast Charge via i3 LIM 
- Nissan Leaf Gen1 inverter via CAN
- Lexus GS450H inverter / gearbox via sync serial
- Toyota Prius/Yaris/Auris Gen 3 inverters via sync serial
- BMW E46 CAN support
- BMW E39 CAN support
- BMW E65 CAN Support

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedencies are libopencm3 and libopeninv. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-vcu by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
