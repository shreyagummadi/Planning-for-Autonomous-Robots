import vrep
import sys
import math


vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print('Connection unsuccessful!')
    sys.exit("Could not connect")
    
    
[s,left_wheel] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")

[s,right_wheel]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")

[s,reference_frame]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")

[s,turtlebot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")

[s,start_position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
if s != 0:
    print("Error")

f = open('x.txt','r')
rx = f.read().splitlines()
rx.reverse()
f1 = open('y.txt','r')
ry = f1.read().splitlines()
ry.reverse()


s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")
s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait)
if s != 0:
       print("Error")

for i in range(1,len(rx)):

        [_,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
        [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
        if s != 0:
            print("Error")
        theta_req = math.atan2(ry[i]-position[1],(float(rx[i])/10)-position[0])
        print(i)

        while  abs(theta[2] - theta_req)>0.5:
            if theta[2] < theta_req:
                s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-2,vrep.simx_opmode_oneshot_wait)
#                print(s)
                if s != 0:
                    print("Error");
                s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait)
                if s != 0:
                    print("Error")
            else:
                s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,2,vrep.simx_opmode_oneshot_wait)
                if s != 0:
                    print("Error")
                s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait)
                if s != 0:
                    print("Error")
                
            [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
            if s != 0:
                print("Error");
        
        [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        if s != 0:
           print("Error")
        position = position
               
        k=1
        while 1:
            k=k+1
            [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
            if s != 0:
               print("Error")
            position = position
                    
            if abs(position[0] - (float(rx[i])/10)) < 0.5 and abs(position[1] - (float(ry[i])/10)) < 0.5 :
                break
            s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,10,vrep.simx_opmode_oneshot_wait)
            if s != 0:
               print("Error")
            s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,10,vrep.simx_opmode_oneshot_wait)
            if s != 0:
               print("Error")

            if k>10:
                print("Here")
                break
 

        print("Location Reached");
        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait)
        if s != 0:
            print("Error")
        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait)
        if s != 0:
            print("Error")
    
vrep.simxFinish(-1)
