# Antenna-Tracker
This project is associated with Aries Group. In this project we have to continuously track a drone using GPS coordinates and point the antenna to the drone by using motors so that that transmission and receiving of data can take place (communication).

FILES- This repository consist of 4 files which are-

1- gps.txt- It is a text file consisitng of different GPGGA strings which act as a like of incoming GPS strings from drone.

2- processing.pde- It is a code written in processing language and will run in processing IDE. It open and reads the "gps.txt" file and extracts latitude, longitude and altitude from the GPGGA strings convert them to degrees and assigns positive and negative value according to its direction. After that it sends latitude, longitude and altitude through serial port to the connected Arduino for further calculations.

3- Antenna_Tracker_Arduino.ino- It is the code which runs on arduino and is written in arduino IDE. It does the major part of calculations and controlling of motors. After receiving the coordinates from processing code it calculates the distance between the GPS coordinates of drone and antenna and calculates the angles which will be given to motors for rotating antenna.

4- Speculation Document- It is the brief explaination of our project with circuit diagrams, code explainations, concepts and formulas used, components used, prerequisites, settings for IDE, challenges etc. For the complete working and running of the project, please read the speculation document for better understanding.

WORKING- First of all run the code on arduino so that it waits for the serial input, then run the processing code which will give the manipulated latitude, longitude and altitude to the arduino which uses them to calculate distance by haversine formula, calculates height, calculates the vertical angle using distance and height, calculates horizontal angle by using bearing angle formula. Here the braud rate of communication of arduino ide and processing ide is same so that no lag in communication between them. After that it gives the vertical angle to a servo motor connected to the arduino and also sends the horizontal angle to a dc motor with encoder which uses a PID algorithm for precise position and speed.

Note- Since the complete project has been done in online mode, there are several points where we have taken some assumptions, and also data that we have used is not quite accurate and doesn't resemble much with real time conditions so there are many points where the data has to be changed according to the situation.
