clear all;
clear; 
clc;
%build map
im_map = imread('map_office.png');
im_bin = im2bw(im_map);
myMaplogical = not(logical(im_bin));
map = binaryOccupancyMap(myMaplogical,100);
show(map);

%create roadmap
PRM = mobileRobotPRM(map,500);
%show(PRM);

%find path
startPositionCoppelia = [2.6 1.5];
goalPositionCoppelia = [2.3 2.6];
startPositionMatlab = posconvert("C2M",startPositionCoppelia);
goalPositionMatlab = posconvert("C2M",goalPositionCoppelia);
pathMatlab = findpath(PRM,startPositionMatlab,goalPositionMatlab);
pathCoppelia = posconvert("M2C",pathMatlab);
show(PRM);
pause(2); %wait 2sec

%pure pursuit
pp = controllerPurePursuit;
pp.Waypoints = pathMatlab;

%braitenberg algorithm
point = [-1 -1 -1 -1 -1 -1 -1 -1];
noDetectionDist=0.27;
maxDetectionDist=0.2;
detect=[0 0 0 0 0 0 0 0];
braitenbergL=[-0.2 -0.4 -0.6 -0.8 -1.0 -1.2 -1.4 -1.6];
braitenbergR=[-1.6 -1.4 -1.2 -1.0 -0.8 -0.6 -0.4 -0.2];
v0=1.25;

%Connect to CoppeliaSim
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

%control loop
if (clientID>-1)
    disp('Connected to remote API server');
    [err_code, right_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    [err_code, left_motor] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [err_code, pioneer_p3dx] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
    for i=1:1:8
        [ret,sensor(i)]=sim.simxGetObjectHandle(clientID,sprintf('Pioneer_p3dx_ultrasonicSensor%d',i),sim.simx_opmode_blocking);
    end
    for i=1:1:8
        [ret,detect(i),distance(i,:),~,~]=sim.simxReadProximitySensor(clientID, sensor(i),sim.simx_opmode_streaming);
    end
    distToGoal = 10; 
    while distToGoal >= 0.1
        %%read sensor 1-8
        for i=1:1:8
            [ret,detect(i),distance(i,:),~,~]=sim.simxReadProximitySensor(clientID, sensor(i),sim.simx_opmode_buffer);
            if(detect(i)==0)
                point(i)=1;
            else
                point(i)=norm(distance(i,:));
            end
        end
        %%pure pursuit
        if(all(point>0.3))
        %Get position and orientation of robot
        [ret_code, pos]=sim.simxGetObjectPosition(clientID,pioneer_p3dx,-1,sim.simx_opmode_blocking);
        [ret_code, ori]=sim.simxGetObjectOrientation(clientID,pioneer_p3dx,-1,sim.simx_opmode_blocking);
        currentPositionCoppelia = pos(1,1:2); 
        currentOrientationCoppelia = ori(1,3); 
        currentPositionMatlab = posconvert("C2M",currentPositionCoppelia); 
        currentOrientationMatlab = oriconvert(currentOrientationCoppelia);
        currentPoseMatlab = [currentPositionMatlab currentOrientationMatlab]; 

        %Connect to CoppeliaSim
        [v,w] = pp(currentPoseMatlab);

        %Connect to CoppeliaSim
        distToGoal = sqrt(sum((currentPositionMatlab-goalPositionMatlab).^2));
        if distToGoal < 0.1
            reachedGoal = 1;
        else
            reachedGoal = 0;
        end 
        vMR = (1-reachedGoal)*v; 
        wMR = (1-reachedGoal)*w;

        %Calculate and set angular velocity for right and left wheel of mobile robot
        [phiR, phiL] = invkinem(vMR,wMR); 
        ret_code = sim.simxSetJointTargetVelocity(clientID, right_motor, phiR, sim.simx_opmode_streaming);
        ret_code = sim.simxSetJointTargetVelocity(clientID, left_motor, phiL, sim.simx_opmode_streaming); 
        
        else
            for i=1:8
                dist=point(i);
                if(dist<noDetectionDist)
                    if(dist<maxDetectionDist)
                        dist=maxDetectionDist;
                    end
                    detect(i)=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
                else
                    detect(i)=0;
                end
            end
            vLeft=v0;
            vRight=v0;
            for i=1:8
                vLeft=vLeft+braitenbergL(i)*detect(i);
                vRight=vRight+braitenbergR(i)*detect(i);
            end
        %Send to motor
            [ret]=sim.simxSetJointTargetVelocity(clientID,left_motor,vLeft,sim.simx_opmode_blocking);
            [ret]=sim.simxSetJointTargetVelocity(clientID,right_motor,vRight,sim.simx_opmode_blocking);    
        end
        
        if(distToGoal < 0.3)
            ret_code = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_blocking);
            ret_code = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_blocking);       
            [ret]=sim.simxAddStatusbarMessage(clientID,'=====FINISH=====',sim.simx_opmode_blocking);
            break;
        end
    end
    disp(distToGoal);
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor! 
disp('Program ended');