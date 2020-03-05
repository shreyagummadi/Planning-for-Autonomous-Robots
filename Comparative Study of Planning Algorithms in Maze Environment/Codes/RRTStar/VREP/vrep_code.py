import vrep
import sys
import numpy as np

text_file = open("FinalPathv.txt", "r")
lines = (text_file.read()).split('  ')
text_file.close()
fpath=[]
for i in lines:
    if(i!=''):
        k=i.split(',')
        for j in range(2):
            k[j]=float(k[j])
    
        fpath.append(k)
print
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print('Connection unsuccessful!')
    sys.exit("Could not connect")
    

errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
errorCodeR,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
returncode,bot = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)

err,simtime = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_streaming)
          
for i in fpath:
        vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,float(i[0]),vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,float(i[1]),vrep.simx_opmode_streaming)
        _,simtime1 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        while (simtime2 - simtime1) < 1.5:
            _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0,vrep.simx_opmode_blocking)
vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0,vrep.simx_opmode_blocking)         
vrep.simxFinish(-1)