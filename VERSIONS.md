## ED BMSdiag version history
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
|         | ... usage of min. data type|
|         | ... new structure typedef and templates|
|         | ... small bugfixes|
|v0.38    | New readings and stability improvements|
|         | ... read HV-Battery voltage even with contactors open|
|         | ... get HV-DC-Isolation in kOhm (reliable with contactors closed)|
|         | ... get info about hardware- / software-revision of battery|
|v0.35a   | Bugfix to continue reading messages if one was skipped|
|         | ... show 0.00kW power instead of -300.00kW|
|v0.35    | Added new readouts|
|         | ... realSOC, current, power, time from BC|
|         | ... EXPERIMENTAL: initial capacity & loss|
|         | Changed data formatting and code cleanup|
|v0.31    | Fixed overflow in ODO calc.|
|         | ... no special ASCII-characters for int. support|
|v0.30    | Added more readout data and reformatted the output|
|         | ... ODO, 12V battery, HV-unit production date|
|         | ... temperatures of battery modules|
|         | ... # of cell at min, max (in individual cell data)|
|v0.24    | Overall improvements|
|         | ... added Doxygen compatible comments|
|         | ... added standard deviation calc.|
|         | ... changed formatting|
|         | Average.h (**please reload library**)|
|         | ... corrected vice versa cell mapping|
|         | ... overflow corrected|
|v0.21    | Waiting for serial init|
|         | ... Showing status of OBD port (CAN-Bus)|
|v0.2     | Initial Commit|
|         | ... Tested with Arduino R3 and Sparkfun CAN-Bus shield|
