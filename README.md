# ED BMSdiag

[![version](https://img.shields.io/badge/version-v0.6.2-blue.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/archive/master.zip)
[![version](https://img.shields.io/badge/issues-none-brightgreen.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/issues)
[![release](https://img.shields.io/badge/release-v0.6.0-orange.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/releases)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/MyLab-odyssey/ED_BMSdiag/blob/master/LICENSE.txt)

Retrieve battery diagnostic data from your smart electric drive EV.
Further documentation in the [Wiki](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki).

---

## You need
An Arduino with CAN bus shield to connect to the diagnostics port (OBDII-connector) of your car. Find information about usable [CAN-Shields here](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/CAN-Bus-Basics). If you have trouble getting a CAN connection please see the [Troubleshooting and Tips](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Troubleshooting-and-Tips) section.

Get the hardware and use an appropriate cable for the physical connection. See the schematics for making your own cable.

<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%26CANbusShield.jpg" width="640"/>
<p/>

## Get started
* Download the [repo-ZIP](https://github.com/MyLab-odyssey/ED_BMSdiag/archive/master.zip), then copy the files to your Arduino folder. The folder structure should look like this:
<p align="left">
<img  src="https://github.com/MyLab-odyssey/ED_BMSdiag/raw/master/pictures/Arduino_folder_structure.png" /><p/>

* Open the ED_BMSdiag.ino file and compile / upload it to the Arduino board.  

* Find detailed installation instructions in: [english](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Installation) | [german](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Installation_DE).

> **Verified to work with Arduino IDE 1.8.1 (on OS X 10.12 and WIN-Systems)**

## Usage >on your own risk<
Connect the CAN shield to the OBDII-connector and power up the car.

This simple tool will display the diagnostics via a serial USB connection. The readout will be started by entering commands after the prompt. See the [Wiki for further details of the CLI](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Command-Line-Interface).


>**You need to open the serial monitor of the Arduino-IDE.  Verify that it is set to CR = Carriage Return and the baud rate is 115200.**

<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%20-IDE_serial_monitor.png" />
<p/>

## Version history
|version  | comment|
|-------- | --------|
|v0.6.2   | Standard charger now implemented as **OBL** command in CLI|
|         | ... use from `CMD >>` prompt with the command `obl all` to get all readouts from the LEAR **O**n**B**oard**L**oader, or use `obl v` for voltages and `obl t` for temperatures on their own. See `?` for help in all menu levels ;-)|
|         | ... THX to "erich" & "AlterAmi" for their support|
|         | ... PLEASE report if values are plausible !!!|
|v0.6.0   | Some new features and minor bug fixes:|
|         | ... make runtime memory more efficient by skipping unused data (>200 bytes saved during cell readout)|
|         | ... show **F**actory **A**cceptance **T**esting date of the completed battery |
|         | ... tested / corrected display of negative cooling-temperatures (done in wintertime)|
|v0.5.6   | Fixed memory leak issue [#8](https://github.com/MyLab-odyssey/ED_BMSdiag/issues/8)|
|v0.5.5   | NLG6 fast charger SW revisions and HW PN now reported (see issue [#7](https://github.com/MyLab-odyssey/ED_BMSdiag/issues/7))|
|         | ... power calculation now updated in BMS submenu ([#5](https://github.com/MyLab-odyssey/ED_BMSdiag/issues/5) fixed)|
|         | ... canDiag-Library updated to real time data of drivetrain ([#6](https://github.com/MyLab-odyssey/ED_BMSdiag/issues/6), not used by now in this project)|
|v0.5.2   | NLG6 fast charger is now detected by hardware part number|
|         | ... power calculation now corrected|
|v0.5.1   | Small bugfix for logging min, max HV-Voltage not updating|
|         | ... added HV-Ri to logging values|
|         | ... new readout for cooling fan (RPM / OTR)|
|v0.5.0   | Added a CLI and new readouts:|
|         | ... NLG6 fast charger|
|         | ... Cooling- and other subsystems|
|         | ... Logging function|
|v0.4.2   | **please reload all files !**|
|         | Bugfix for Average-Template memory bug > rewritten  to AvgNew-Class|
|         | ... This version uses dynamic memory allocation, so monitor free memory (between heap and stack) if you make modifications!|
|v0.4.0   | New canDiag class structure and BMS_dfs.h for BMS specific code|
|v0.3.9b  | **now using semantic versioning** (old files unchanged)|
|v0.39b   | fixed type cast bug HVcontactor-timer|
|v0.39    | Optimized memory usage to gain ~ 560 Bytes of SRAM for more features.|

(For more entries see VERSIONS.md)
