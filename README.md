maslab-staff-2013
=================

MASLAB Staff-provided code for IAP 2013.

Detailed breakdown by folder:
* arduino_firmware - contains all code related to loading firmware onto the Arduino
    * arduino_firmware.ino - the Arduino sketch file containing the main firmware code
    * sketchbook - the sketchbook folder that the Arduino software should point to in order to load the necessary libraries.
* arduino.py - the Python interface for communicating with the Arduino. Initialize classes and use class methods to interact with the Arduino. This file spawns a new thread that constantly communicates with the Arduino based on values in arrays set by the class methods.
* examples - contains several code examples for interfacing with the given libraries.
    * actuators - code examples related to motors, servos, steppers, etc.
    * sensors - code examples related to IR sensors, bump sensors, etc.
    * general - general code examples