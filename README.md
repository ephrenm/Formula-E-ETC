# Formula E ETC
 [TODO]
 Add check for enable signal from master controller before outputting to I2C
 
 Add code to set a digital pin high whenever brake pedal is pressed (brake light activation for master controller)
 
 Out-of-range values from APPS to Arduino & between Arduino and motor controller - T.4.2.10 & T.4.3.4
 
 Test calibration in assembly
 
 Add check to calibration numbers before writing to EEPROM
 
 Add check to only allow calibration while accel < 10 (creates issues if throttle stuck > 10, maybe only while moving? or allow when accel > 10 and break engaged?)
 
 Improve Readibility: Add more comments, but variable names
