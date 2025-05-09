# stm32-vcu (ZombieVerter VCU)

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

Project based on the OpenInverter System by Johannes Huebner to provide a universal VCU (Vehicle Control Unit) for electric vehicle conversion projects. 

Please visit the development thread on the Openinverter Forum for more information : https://openinverter.org/forum/viewtopic.php?f=3&t=1277
![Screenshot from 2025-01-28 14-17-28](https://github.com/user-attachments/assets/ff066c9e-8c79-470d-aa04-bc3b34198900)

## Videos on progress

Project introduction : https://vimeo.com/506480876

V1.11a On dyno with GS450h / BMW E39 : https://vimeo.com/802405172

V2.00a now available : https://vimeo.com/824494783?share=copy

V2.15a now available : https://www.youtube.com/watch?v=iwB3wxCEFo0

V2.20A now available : https://www.youtube.com/watch?v=wjlucUWX_lc

# Supported components

- Nissan Leaf Gen1/2/3 inverter via CAN
- Nissan Leaf Gen1/2/3 PDM (Charger and DCDC)
- Nissan LEAF Battery (all variants)
- Mitsubishi Outlander Support
- Mitsubishi Outlander drivetrain (front and rear motors/inverters) Support
- Modular BMS / SimpBMS Support
- OpenInverter CAN Support
- CCS DC fast charge via BMW i3 LIM
- CCS DC fast charging via FOCCCI https://github.com/uhi22/ccs32clara
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

# Contributing

Information on how to contribute to this open source project can be found [Contributing Guide](./CONTRIBUTING.md)

# All hardware / software is tested in these vehicles before release. More vehicles with different configuration in progress to ensure as much testing as possible.
Note: Only RELEASES are tested. Code in repository is to be treated as unproven and used as such
