# METR4901

Contains all work material for METR4901 thesis on Robotic Arm simulations in VREP.

The remote executable is stored in /programming/client/x64/Release/vrepClientProgram.exe
Usage: vrepClientProgram.exe ik/fk [object filename] [IP Adress] [port]
by default the IP Address and port are local host and 19999.

ik and fk scenes containing the Jaco arm are found in the route folder.

The joystick library used is SDL2-2.0.7, which is open source and has been modified for use.

Only the required VREP remote API files are included in this repository.

The object file is found in the release folder, if it is not there it will be automatically
generated on start-up, which takes ~10 seconds to get all the information from VREP.

_______________________________________________________________________________
Joystick for IK is the only functionality so far, although awsd commands are to come.
Keyboard commands are accepted for FK, in the form: jointNumber Angle\n
Where jointNumber correspondes to joint 1 (base) to 6 (tip).
