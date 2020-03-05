import vrep
import sys



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
f = open('ur.txt','r')
ur = f.read().splitlines()
ur.reverse()
f1 = open('ul.txt','r')
ul = f1.read().splitlines()
ul.reverse()
for i in range(len(ul)):
        vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,float(ul[i]),vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,float(ur[i]),vrep.simx_opmode_streaming)
        _,simtime1 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
        while (simtime2 - simtime1) < 1.5:
            _,simtime2 = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_buffer)
vrep.simxFinish(-1)