clc
clear
close all

vrep=remApi('remoteApi');
vrep.simxFinish(-1); % close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected');

    % Generate object handles from vrep
    [~,left_wheel] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot_wait);

    [~,right_wheel]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot_wait);

    % Taking Dummy as reference frame origin axis
    [~,reference_frame]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_oneshot_wait);
    [~,turtlebot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_oneshot_wait);

    % Main code
    % Get position
    [~,start_position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
    start = double([start_position(1,1),start_position(1,2)]);
    
    % Call Main function
    final_path = DynamicMazeAstar(start(1),start(2));
    final_path = final_path/10;
 
    vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);
    vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);

    for i=2:size(final_path,1)

        [~,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        [~,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);   
        position = double(position);
        theta_req = atan2(final_path(i,2)-position(1,2),final_path(i,1)-position(1,1));
        
        % Orienrtation correction
        while  abs(theta(3) - theta_req)> 0.05
            if theta(3) < theta_req 
                vrep.simxSetJointTargetVelocity(clientID,left_wheel,-2,vrep.simx_opmode_oneshot_wait);
                vrep.simxSetJointTargetVelocity(clientID,right_wheel,2,vrep.simx_opmode_oneshot_wait);
               
            else
                vrep.simxSetJointTargetVelocity(clientID,left_wheel,2,vrep.simx_opmode_oneshot_wait);
                vrep.simxSetJointTargetVelocity(clientID,right_wheel,-2,vrep.simx_opmode_oneshot_wait);
            end
                
            [~,theta]=vrep.simxGetObjectOrientation(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
         
        end
        
        [~,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
        position = double(position);
        
        
        k=1;
        % Position Correction
        while 1
            k=k+1;
            [~,position]=vrep.simxGetObjectPosition(clientID, turtlebot,reference_frame,vrep.simx_opmode_oneshot_wait);
            position = double(position);
                       
            if abs(position(1,1) - final_path(i,1)) < 0.1 &&  abs(position(1,2) - final_path(i,2)) < 0.1
                break;
            end
    
            vrep.simxSetJointTargetVelocity(clientID,left_wheel,10,vrep.simx_opmode_oneshot_wait);
            vrep.simxSetJointTargetVelocity(clientID,right_wheel,10,vrep.simx_opmode_oneshot_wait);
           
            % Orientation condition
            if k>10
                break;
            end
        end
    end
 
    % Stop robot when goal is reached
    disp("Goal Reached!");
    vrep.simxSetJointTargetVelocity(clientID,left_wheel,0,vrep.simx_opmode_oneshot_wait);
    vrep.simxSetJointTargetVelocity(clientID,right_wheel,0,vrep.simx_opmode_oneshot_wait);
    vrep.simxFinish(-1);
end
vrep.delete();