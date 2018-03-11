# METR4901

Contains all work material for METR4901 thesis on Robotic Arm simulations in VREP.

The remote executable is stored in /programming/client/x64/Release/vrepClientProgram.exe
Usage: vrepClientProgram.exe ik/fk [object filename] [IP Adress] [port]
by default the IP Address and port are local host and 19999.

ik and fk scenes containing the Jaco arm are found in the route folder.

The joystick library used is SDL2-2.0.7, which is open source and has been modified for use.

Only the required VREP remote API files are included in this repository.

The 'object filename' is called object.txt, which is found in the release folder, if 'object filename' is not there it will be automatically generated on start-up, which takes ~10 seconds to get all the information from VREP.

_______________________________________________________________________________
Can determine whether a joystick is connected and to use that, or aswd commands, on start-up.
Keyboard commands are accepted for FK, in the form: jointNumber Angle\n, and 'w\n' for forwards in ik.
Where jointNumber correspondes to joint 1 (base) to 6 (tip).

Bugs: deleting objects from a scene will decrease the object count but not modify the objects
handles, so if there's 100 objects, some may have handles >100. Thus loading in the handles is required
for modified scenes, where loading from a file fails due to accessing elements > object count.
