# stm32-vcu

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

Project based on the OpenInverter System by Johannes Huebner to provide a universal VCU (Vehicle Control Unit) for electric vehicle conversion projects. 

Please visit the development thread on the Openinverter Forum for more information : https://openinverter.org/forum/viewtopic.php?f=3&t=1277


Video on progress : https://vimeo.com/506480876

V1.11a On dyno with GS450h / BMW E39 : https://vimeo.com/802405172

V2.00a now available : https://vimeo.com/824494783?share=copy

V2.05a now available.

# Features

- Nissan Leaf Gen1/2/3 inverter via CAN
- Nissan Leaf Gen2 PDM (Charger and DCDC)
- Mitsubishi Outlander Support
- Mitsubishi Outlander drivetrain (front and rear motors/inverters) Support
- Modular BMS / SimpBMS Support
- OpenInverter CAN Support
- CCS DC fast charge via BMW i3 LIM
- Chademo DC fast charge
- ISA Shunt / BMW SBOX / VW EBOX supported via CAN
- Lexus GS450H inverter / gearbox via sync serial
- Lexus GS300H inverter / gearbox via sync serial
- Toyota Prius/Yaris/Auris Gen 3 inverters via sync serial
- BMW E46 CAN support
- BMW E39 CAN support
- BMW E65 CAN Support
- BMW E31 CAN Support
- Mid 2000s VAG Can support
- Subaru vehicle support
- Opel Ampera / Chevy Volt 6.5kw cabin heater
- VW LIN based 6.5kw cabin heater
- Elcon charger Support
- OBD2 Can support
- TESLA Gen 2 DCDC Converter Can support


# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Linux this can be installed by typing:

`sudo apt install gcc-arm-none-eabi`

The only external depedencies are libopencm3 and libopeninv. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-vcu by typing

`make`

# Tests

Build the tests

`make Tests`

Run the tests

`./test/test_vcu`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface

I use CodeBlocks IDE :  https://www.codeblocks.org/

Sept 21 : V1 hardware and firmware from "Master" branch now running in my E46 Touring and E39 vehicles.

March 24 : V1.1 Hardware and V2.05A firmware now running in 3 vehicles :

-BMW E39 with Lexus GS450H Drivetrain, Tesla PCS, ISA Shunt , Chademo Fast Charge

-BMW E46 Touring with Nissan Leaf Gen 1 Drivetrain, Outlander Charger / DCDC, ISA Shunt , CCS Fast Charge

-BMW E31 with Tesla LDU Drivetrain, Tesla DCDC, ISA Shunt , Chademo Fast Charge

All hardware / software is tested in these vehicles before release. More vehicles with different configuration in progress to ensure as much testing as possible.
