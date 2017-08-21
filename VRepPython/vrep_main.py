# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import vrep
import sys

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
#	number clientID=simxStart(string connectionAddress,number connectionPort,boolean waitUntilConnected,boolean doNotReconnectOnceDisconnected,number timeOutInMs,number commThreadCycleInMs)

if clientID >= 0:
        ## successfully connected to API server
        print "successfully connected to V-REP API server"
else:
    print "connection failed"
    sys.exit('error, could not connect')
    quit()
    
# get handle on component to move, i.e. j1=simGetObjectHandle("JacoHand_fingers12_motor2")
# if object does not exist, error code 8 and handle of 0
    
## gets handle of jaco hand joints
errorCode, motor1handle = vrep.simxGetObjectHandle(clientID, "JacoHand_fingers12_motor1", vrep.simx_opmode_oneshot_wait)
errorCode, motor2handle = vrep.simxGetObjectHandle(clientID, "JacoHand_fingers12_motor2", vrep.simx_opmode_oneshot_wait)
## gets handle of jaco arm joints, one at a time
errorCode, joint1handle = vrep.simxGetObjectHandle(clientID, "Jaco_joint1", vrep.simx_opmode_oneshot_wait)
errorCode, joint2handle = vrep.simxGetObjectHandle(clientID, "Jaco_joint2",
                                                   vrep.simx_opmode_oneshot_wait)

print "getting joint information"
## gets force and position of joints
errorCode1, initPosition1 = vrep.simxGetJointPosition(clientID, joint1handle, vrep.simx_opmode_oneshot_wait)
errorCode1, initForce1 = vrep.simxGetJointForce(clientID, joint1handle, vrep.simx_opmode_oneshot_wait)
errorCode2, initPosition2 = vrep.simxGetJointPosition(clientID, joint2handle, vrep.simx_opmode_oneshot_wait)
errorCode2, initForce2 = vrep.simxGetJointForce(clientID, joint2handle, vrep.simx_opmode_oneshot_wait)

errorCode1, initPosition4 = vrep.simxGetJointPosition(clientID, motor1handle, vrep.simx_opmode_oneshot_wait)

print "set joint information"
## pause stream and prepare dataload to send
vrep.simxPauseCommunication(clientID, True)
errorCode2 = vrep.simxSetJointTargetVelocity(clientID, joint1handle, 0.2, vrep.simx_opmode_oneshot) 
errorCode2 = vrep.simxSetJointForce(clientID, joint1handle, 15, vrep.simx_opmode_oneshot)
errorCode2 = vrep.simxSetJointTargetVelocity(clientID, joint2handle, 0.2, vrep.simx_opmode_oneshot) 
errorCode2 = vrep.simxSetJointForce(clientID, joint2handle, 15, vrep.simx_opmode_oneshot)
## send data stream
vrep.simxPauseCommunication(clientID, False)

print "move arms"
## move arms
errorCode2 = vrep.simxSetJointTargetPosition(clientID, joint2handle, 200*3.14195/180, vrep.simx_opmode_oneshot_wait) 
errorCode3, Position4 = vrep.simxGetJointPosition(clientID, joint2handle, vrep.simx_opmode_oneshot_wait)
targetVelMotor1 = 0
if initPosition4 > 0.0:
    targetVelMotor1 = -0.2
else:
    targetVelMotor1 = 0.2
errorCode4 = vrep.simxSetJointTargetVelocity(clientID, motor1handle, targetVelMotor1, vrep.simx_opmode_streaming)

vrep.simxFinish(clientID);

print "code ended"