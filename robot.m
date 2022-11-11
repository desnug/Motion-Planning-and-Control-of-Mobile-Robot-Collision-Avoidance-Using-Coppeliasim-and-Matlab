%Make map and path plan
%               1          5           10           15           20           25           30
myMap = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;...
                  1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1;...%1
                  1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...
                  1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...%2
                  1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...
                  1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...%3
                  1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...
                  1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...%4
                  1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1;...
                  1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1;...%5
                  1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1;...
                  1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 1 1 1 1;...%6
                  1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 0 1 1 1 1 1 1 1 1 1 1 1;...
                  1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1;...%7
                  1 0 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1;...
                  1 0 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1;...%8
                  1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1;...
                  1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 1 1 0 1 1 1 1 1 1 1 1 1 1 1;...%9
                  1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1;...
                  1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1;...%10
                  1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1;...
                  1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...%11
                  1 1 1 0 1 0 0 0 1 0 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...
                  1 1 1 1 1 0 0 0 1 1 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...%12
                  1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...
                  1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1;...%13
                  1 1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...
                  1 0 1 0 1 1 0 0 0 1 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...%14
                  1 1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 0 0 1 1 1;...
                  1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];  %15
myMaplogical = logical(myMap);
map = binaryOccupancyMap(myMaplogical,2);
map.GridOriginInLocal = -[diff(map.XWorldLimits) diff(map.YWorldLimits)]/2;
prmSimple = mobileRobotPRM(map,500);
% show(prmSimple);
startPosition = [-4 -6.5];
goalPosition = [-4 1.5];
path = findpath(prmSimple,startPosition,goalPosition);
show(prmSimple);
pause(8); %wait 8sec
pp = controllerPurePursuit('DesiredLinearVelocity',1.2,'MaxAngularVelocity',0.06);
pp.Waypoints = path;

%initial position and speed
currentPosition = path(1,:); % start position
currentOrientation = pi/180*90; % 90
currentPose = [currentPosition currentOrientation];
[v,w,npos] = pp(currentPose);
theta = atan2((npos(2)-currentPose(2)),(npos(1)-currentPose(1)));
dtheta=theta-currentPose(3);
pos = [0 0 0]; %x,y,A
R = 0.195/2;
L = 0.415/2;
a = [ v*cos(dtheta); -w];
b = [  R/2    R/2;
     R/2*L -R/2*L];
b_inv = inv(b);
vout = b_inv*a*R;

%braitenberg
point = [-1 -1 -1 -1 -1 -1 -1 -1];
noDetectionDist=0.27;
maxDetectionDist=0.15;
detect=[0 0 0 0 0 0 0 0];
braitenbergL=[-0.2 -0.4 -0.6 -0.8 -1.0 -1.2 -1.4 -1.6];
braitenbergR=[-1.6 -1.4 -1.2 -1.0 -0.8 -0.6 -0.4 -0.2];
v0=1.25;

%start remote api
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if(clientID>-1)
    disp('connected')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WRITE CODE HERE: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %setup all handle
    [ret,left_motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [ret,right_motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    for i=1:1:8
        [ret,sensor(i)]=sim.simxGetObjectHandle(clientID,sprintf('Pioneer_p3dx_ultrasonicSensor%d',i),sim.simx_opmode_blocking);
    end
    [ret,p3dx_position]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
    [ret,p3dx_orientation]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
    %setup sensor,position,orientation  
    for i=1:1:8
        [ret,detect(i),distance(i,:),~,~]=sim.simxReadProximitySensor(clientID, sensor(i),sim.simx_opmode_streaming);
    end
    [ret,position]=sim.simxGetObjectPosition(clientID,p3dx_position,-1,sim.simx_opmode_streaming);
    [ret,orientation]=sim.simxGetObjectOrientation(clientID,p3dx_orientation,-1,sim.simx_opmode_streaming);
    
    %Main loop
    while 1
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
            [v,w,npos] = pp(currentPose);
            theta = atan2((npos(2)-currentPose(2)),(npos(1)-currentPose(1)));
            dtheta=theta-currentPose(3);
            a = [v*cos(dtheta); -w];
            vout = b_inv*a*R;
            [ret]=sim.simxSetJointTargetVelocity(clientID,left_motor,vout(1,:),sim.simx_opmode_blocking);
            [ret]=sim.simxSetJointTargetVelocity(clientID,right_motor,vout(2,:),sim.simx_opmode_blocking);
            %%get position and orientation p3dx
            [ret,position]=sim.simxGetObjectPosition(clientID,p3dx_position,-1,sim.simx_opmode_buffer);
            [ret,orientation]=sim.simxGetObjectOrientation(clientID,p3dx_orientation,-1,sim.simx_opmode_buffer);
            pos = [position(1) position(2) orientation(3)];
            currentPose = double(pos);
        %braitenberg
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
        
        %%cal dis goal
        distToGoal = sqrt(sum((currentPose(1:2)-goalPosition).^2))
%         disp(distToGoal)
        if(distToGoal < 0.3)
            [ret]=sim.simxSetJointTargetVelocity(clientID,left_motor,0,sim.simx_opmode_blocking);
            [ret]=sim.simxSetJointTargetVelocity(clientID,right_motor,0,sim.simx_opmode_blocking);
            [ret]=sim.simxAddStatusbarMessage(clientID,'=====FINISH=====',sim.simx_opmode_blocking);
            break;
        end 
    end
    
    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);
end
sim.delete();