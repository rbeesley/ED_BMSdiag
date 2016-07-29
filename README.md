# ED BMSdiag
Retrieve battery diagnostic data from your smart electric drive EV.



### You need
An Arduino with CAN bus shield to connect to the diagnostics port (OBDII-connector) of your car. Get the hardware and use an appropriate cable for the physical connection. See the schematics for making your own cable.

<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%26CANbusShield.jpg" width="640"/>
<p/>

### Get started
* Download the repo-ZIP, then copy the files to your Arduino folder. The folder structure should look like this:
<p align="left">
<img  src="https://github.com/MyLab-odyssey/ED_BMSdiag/raw/master/pictures/Arduino_folder_structure.png" /><p/>

* Open the ED_BMSdiag.ino file and compile / upload it to the Arduino board.

Please see the [wiki](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki) for further information.

### Usage [>on your own risk<]
Connect the CAN shield to the OBDII-connector and power up the car.

This simple tool will display the diagnostics via a serial USB connection. The readout will be started by entering commands after the prompt. See the [wiki for further details of the CLI](https://github.com/MyLab-odyssey/ED_BMSdiag/wiki/Command-Line-Interface).


>**You need to open the serial monitor of the Arduino-IDE.  Verify that it is set to CR = Carriage Return and the baud rate is 115200.**

<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%20-IDE_serial_monitor.png" />
<p/>

### Version history
version  | comment
-------- | --------
v0.5.0   | Added a CLI and new readouts:
         | ... NLG6 fast charger
         | ... Cooling- and other subsystems
         | ... Logging function
v0.4.2   | **please reload all files !**
         | Bugfix for Average-Template memory bug > rewritten  to AvgNew-Class
         | ... This version uses dynamic memory allocation, so monitor free memory (between heap and stack) if you make modifications!
v0.4.0   | New canDiag class structure and BMS_dfs.h for BMS specific code
v0.3.9b  | **now using semantic versioning** (old files unchanged)
v0.39b   | fixed type cast bug HVcontactor-timer
v0.39    | Optimized memory usage to gain ~ 560 Bytes of SRAM for more features.
(For more entries see VERSIONS.md)
