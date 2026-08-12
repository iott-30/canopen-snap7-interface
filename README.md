## Overview

The main goal of this project is bidirectional communication between a PC and CANopen device with a S7-1200 PLC as the mediator/middleman. The PLC uses the HMS Ixxat CM CANopen module to interface with the CANopen network. The setup currently transmits generic 8-bit and 16-bit simulated input/output values.
- **The PC** runs a python program using the [python-snap7](https://github.com/gijzelaerr/python-snap7) library to communicate with the PLC via LAN. The program allows user input to change one of the values, `SlaveAnalogOutput1`, during runtime (with the rest being configurable in the code), and changes in the input DB (CAN process image) are reported in the console.
- **The PLC** receives traffic from the two networks, maps the data to their respective DBs (to be accessable from blocks within the PLC), transmits the data to the CAN module, and allows the PC to read the data from the DBs.
- **The CANopen device** is being simulated with a python program on a PC that is connected to the network via a RH02 USB to CAN adapter. The program uses the [python-canopen](https://github.com/canopen-python/canopen) library. User input for `SlaveAnalogInput1` is allowed during runtime, and the other values can be set in the code beforehand. Changes to the outputs are reported in the console. Data is transmitted with PDOs (`RPDO1`, `RPDO2`, `TPDO1`, and `TPDO2`), and both the CANopen simulated slave device and CANopen manager (CM module) have heartbeats.

Since the data in the PLC is asynchronously accessed by the PC, double buffer and/or handshake functionality was implemented to prevent torn reads/writes to the input and output DBs.

## References

- Code blocks for the CANopen functionality on the PLC are based on the [CANopen with SIMATIC S7](https://support.industry.siemens.com/cs/ww/en/view/109479771) application example provided by Siemens.
- [Ixxat CM CANopen support and downloads](https://www.hms-networks.com/p/021620-b-ixxat-cm-canopen?tab=tab-support)
