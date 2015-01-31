2012MASLabTeam6
=================

Code for the robot for MASLab Team 6, based on MASLAB Staff-provided code for IAP 2013.

To run, load the arduino firmware onto an arduino and connect it to the robot's computer. On the robot's computer, run python mock3.py, which is the code for the mock competition. This software requires openCV.

Detailed breakdown by folder:
* arduino_firmware - contains all code related to loading firmware onto the Arduino
    * arduino_firmware.ino - the Arduino sketch file containing the main firmware code
    * sketchbook - the sketchbook folder that the Arduino software should point to in order to load the necessary libraries.
* arduino.py - the Python interface for communicating with the Arduino. Initialize classes and use class methods to interact with the Arduino. This file spawns a new thread that constantly communicates with the Arduino based on values in arrays set by the class methods.
* examples - contains several code examples for interfacing with the given libraries.
    * actuators - code examples related to motors, servos, steppers, etc.
    * sensors - code examples related to IR sensors, bump sensors, etc.
    * general - general code examples
* MASLab simulation - a simulation for a robot exploring a field for use in testing
