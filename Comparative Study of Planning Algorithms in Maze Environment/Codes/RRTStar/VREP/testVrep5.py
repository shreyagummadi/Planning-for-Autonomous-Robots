import vrep
import sys
import math
import numpy as np

vrep.simxFinish(-1)
r=[[1.5, 6.8], [1.4331414059619192, 6.868538765892675], [-1.6064300122745405, 5.986789631594058], [-2.0092298657086225, 5.614086365025141], [-2.686519892090655, 4.897192170902571], [-3.0950828616208685, 4.492526509371485], [-4.770610672014685, 2.573660911430492], [-5.8039380948073624, 1.311042619328609], [-6.792951845413683, 0.03204117765496167], [-9, -1]]
#r.reverse()
text_file = open("x.txt", "w")
text_file1 = open("x.txt", "w")
for i in range(len(r)):
    r[i][0]=(r[i][0]+12.5)*10
    r[i][1]=(r[i][1]+7.5)*10
    text_file.write(str(r[i][0])+'\n')
    text_file1.write(str(r[i][1])+'\n')
#print(r)
text_file.close()
text_file1.close()
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print('Connection unsuccessful!')
    sys.exit("Could not connect")
    

#errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
#errorCodeR,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
#returncode,bot = vrep.simxGetObjectHandle(clientID,'Turtlebot2_target', vrep.simx_opmode_oneshot_wait)
#err,simtime = vrep.simxGetFloatSignal(clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_streaming)
##returnCode = vrep.simxSetObjectPosition(clientID,bot,h,-1,vrep.simx_opmode_oneshot)
#returnCode=vrep.simxSetObjectPosition(clientID,bot,-1,[100,100,0],vrep.simx_opmode_buffer)
#
#
#vrep.simxFinish(-1)
    
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

    

s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,1,vrep.simx_opmode_oneshot_wait)
if s != 0:
        print("Error")
s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,0.5,vrep.simx_opmode_oneshot_wait)
if s != 0:
       print("Error")

for i in range(1,len(r)):
        print(r[i])
        [_,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
        [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
        if s != 0:
            print("Error")
        theta_req = math.atan2(r[i][1]-position[1],r[i][0]-position[0])
#        print('i',i)

        while  abs(theta[2] - theta_req)> 0.5:
#            print(theta[2],' ', theta_req,' ,diff',abs(theta[2] - theta_req))
            if theta[2] < theta_req:
                    if(np.abs(theta[2] - theta_req)<1):
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-1,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,1,vrep.simx_opmode_oneshot_wait);

                    else:
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,-2,vrep.simx_opmode_oneshot_wait);

                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait);
               
                    if s != 0:
                        print("Error");
                    s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait)
                    if s != 0:
                     print("Error")
            else:
                if(np.abs(theta[2] - theta_req)<1):
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,1,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-1,vrep.simx_opmode_oneshot_wait);

                else:
                        s = vrep.simxSetJointTargetVelocity(clientID,left_wheel,2,vrep.simx_opmode_oneshot_wait);
                        s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait);
                if s != 0:
                    print("Error")
                s = vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait)
                if s != 0:
                    print("Error")
              
            [s,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
            if s != 0:
                print("Error");
#        print(abs(theta[2] - theta_req)) 
        [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        if s != 0:
           print("Error")
               
        k=1
        while 1:
            k=k+1
            [s,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait)
            if s != 0:
               print("Error")
                      
            if abs(position[0] - r[i][0]) < 0.5 and abs(position[1] - r[i][1]) < 0.5 :
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
    
vrep.simxFinish(-1);
