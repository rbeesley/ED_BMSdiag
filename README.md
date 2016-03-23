# ED BMSdiag
Retrieve battery diagnostic data from your smart electric drive EV.

 <p align="center">
 <img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/ED_BMSdiag_example_data.png?token=ANltnDv45WNbnI5tdK5GTXsswYPnh61kks5W8nFpwA%3D%3D" />
 <p/>

## You need
An Arduino with CAN bus shield to connect to the diagnostics port (OBDII-connector) of your car. Get the hardware and use an appropriate cable for the physical connection. See the schematics for making your own cable.

<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%26CANbusShield.jpg?token=ANltnOgbz5LLS_DZFdx2z4qH653Vq6GQks5W8nDIwA%3D%3D" width="640"/>
<p/>

## Get started
Download the repo-ZIP, then

* Copy the library-subfolders to your Arduino-library folder.
* Copy the ED_BMSdiag-Folder to the Arduino-sketches folder. If the Arduino-IDE was running - restart it.
* Open the ED_BMSdiag.ino file and compile / upload it to the Arduino board.

## Usage (on your own risk!)
Connect the CAN shield to the OBDII-connector and power up the car.

This simple version will display the diagnostics via a serial USB connection. You need to open the serial monitor of the Arduino-IDE. Verify that the baud rate is set to 115200. **The readout will be started by entering a serial command / keystroke in the top input line.** You will get an text output of the current battery status.
<p align="center">
<img  src="https://raw.githubusercontent.com/MyLab-odyssey/ED_BMSdiag/master/pictures/Arduino%20-IDE_serial_monitor.png?token=ANltnE8Q_Cad764BoBn_TDdUdreMZXXiks5W8nT6wA%3D%3D" />
<p/>

## Version history
version  | comment
-------- | ---------
v0.2     | Initial Commit
         | Tested with Arduino R3 and Sparkfun CAN-Bus shield
v0.21    | Waiting for serial init
         | Showing status of OBD port (CAN-Bus)
