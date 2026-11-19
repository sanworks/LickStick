# LickStick
A capacitive lick sensing device designed for Neuroscience research.

## Important Note ##
LickStick is in beta testing, and may still have unresolved issues.

## Contents ##
This repository includes:
* Circuit board and enclosure design files
* Link to Bill of Materials
* Firmware (Arduino language, requires Teensy board package)
* A MATLAB class to control the LickStick device
* A Python class to control the LickStick device (initial draft)

## Documentation ##
* [USING.md](USING.md): using the device from MATLAB and Python. Setup, standard calls, parameters, the recorded data and pitfalls
* [AGENTS.md](AGENTS.md): developer guide to this repository, for people and for AI coding agents
* [Firmware/PROTOCOL.md](Firmware/PROTOCOL.md): the USB serial protocol

## Setup ##
* Add /LickStick/MATLAB/ to the MATLAB path
* Connect a LickStick device to your PC
* See comments in LickStick.m for usage
