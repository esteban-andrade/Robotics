%% Collision Checking (use LinePlaneIntersection.m)
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%Create a puma 560.
mdl_puma560;
%The robot is at q = [pi/20,0,-pi/2,0,0,0].
q = [pi/6,0,-pi/2,0,0,0]%Q INPUT
%Determine where a ray cast from the Z axis (the approach vector) of the end effector intersects with a planar wall. (i.e. normal = [-1 0 0], point = [1.2 0 0]).
endEffectorTr = p560.fkine(q)
p560.teach(q)
hold on
 normal = [-1,0,0] ;%Q INPUT
 point= [3.6,0,0] ;%Q INPUT
planeXntersect = 3.6;%Q INPUT
planeBounds = [planeXntersect-eps,planeXntersect+eps,-2,2,-2,2]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect,size(Y,1),size(Y,2));
surf(X,Y,Z);
rayEndTr = endEffectorTr * transl(0,0,10);
point1OnLine = endEffectorTr(1:3,4)';
point2OnLine = rayEndTr(1:3,4)';
[intersectionPoint,check] = LinePlaneIntersection(...
    normal,point,point1OnLine,point2OnLine)

%% Collision Checking (use LinePlaneIntersection.m)
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
%Create 3-link 3D robot with mdl_3link3d.
mdl_3link3d;
q = [-pi/10,0,0]
endEffectorTr = R3.fkine(q)
R3.teach(q)
hold on
planeXntersect = 5;
planeBounds = [planeXntersect-eps,planeXntersect+eps,-3,3,-3,3]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect,size(Y,1),size(Y,2));
surf(X,Y,Z);
point1OnLine = endEffectorTr(1:3,4)';
point2OnLine = [0 0 1];
normal = [-3 0 0]
point = [5   0 0]
[intersectionPoint,check] = LinePlaneIntersection(...
    normal,point,point1OnLine,point2OnLine)



%% Create 5DOF Planar
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
%Make a 5DOF planar.
L1 = Link('d',0,'a',1);
L2 = Link('d',0,'a',1);
L3 = Link('d',0,'a',1);
L4 = Link('d',0,'a',1);
L5 = Link('d',0,'a',1);
        
fiveDOF = SerialLink([L1 L2 L3 L4 L5],'name','5DOF');
       
q = deg2rad([60 -60 60 -60 0]);
%qzero = zeros(1,5)
endEffectorTr = fiveDOF.fkine(q)
% fiveDOF.teach(q)

%% Distance Sense Distance to Puma End Effector
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
mdl_puma560;
p560.base = transl(0,0,0);
q = deg2rad([0, 45, -80, 0, 45, 0]); % Q INPUT
endEffectorTr = p560.fkine(q);
X = 1
Y = -0.6828
Z = 1

distanceSensorTr = transl(X,Y,Z)
% %p560.teach(q)
dist = sqrt((distanceSensorTr(1,4) - endEffectorTr(1,4))^2 + ...
    (distanceSensorTr(2,4) - endEffectorTr(2,4)^2) + ...
    (distanceSensorTr(3,4) - endEffectorTr(3,4))^2)
y = sqrt(2^2-(1-endEffectorTr(1,4))^2-(1-endEffectorTr(3,4))^2)+endEffectorTr(2,4)


%% Lab Assignment 1
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc


%% Point In Puma End Effector Coordinate Frame
%Week Point in Global Coordinate Frame
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
%Create puma 560 with mdl_puma560.
mdl_puma560;
%Assume it is on the floor with q = [90, 30, -80, 0, 45, 0] degrees.
% q = deg2rad([-90, 30, -80, 0, 45, 0]);
q = deg2rad([90, 30, -80, 0, 50, 0]);
endPose = p560.fkine(q) %gets the end effector pose
p560.teach(q); %show the robot position
p560.base = p560.base*transl(0,0,0);
hold on;
%Ball has center defined by global transform transl(0.5,0.1,0.6)*trotx(pi/2).
% ballPose = transl(0.5,0.1,0.6) * trotx(pi/2)  
ballPose = transl(0.5,0.1,0.6) * trotx(pi/2) 
trplot(ballPose);
%What is the balls pose with respect to the end-effectors coordinate frame?
%To get an object2's pose with respect to object1, object2from1pose = inv(object1pose)*object2pose.
endToBallPose = inv(endPose)*ballPose %answer

%% Puma Ikine
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
% Create Puma
mdl_puma560;

%End effector pos= [0.6 -0.1 -0.2]
endEffectorTr = transl(0.7,0.1,0.2)

q = p560.ikine(endEffectorTr,qn, [1 1 1 0 0 0])   


%% Puma Distance To Wall Along Z
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
% Create a puma 560.
mdl_puma560;
%The robot is at q = [pi/20,0,-pi/2,0,0,0].
q = [-pi/10 0 -pi/2 0 0 0];
%Determine where a ray cast from the Z axis (the approach vector) of the end effector intersects with a planar wall. (i.e. normal = [-1 0 0], point = [1.2 0 0]).
endEffectorTr = p560.fkine(q)
p560.teach(q)
hold on
normal = [-1 0 0];
point = [1.2 0 0];
planeXntersect = 1.2;
planeBounds = [planeXntersect-eps,planeXntersect+eps,-2,2,-2,2]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect,size(Y,1),size(Y,2));
surf(X,Y,Z);
rayEndTr = endEffectorTr * transl(0,0,10);
point1OnLine = endEffectorTr(1:3,4)';
point2OnLine = rayEndTr(1:3,4)';
[intersectionPoint,check] = LinePlaneIntersection(...
    normal,point,point1OnLine,point2OnLine)
dist = sqrt((intersectionPoint(1) - endEffectorTr(1,4))^2 + ...
    (intersectionPoint(2) - endEffectorTr(2,4)^2) + ...
    (intersectionPoint(3) - endEffectorTr(3,4))^2)

%% Safety (x2 questions)
%Watch the recommended videos. Read the prescribed textbook pages.
%When is it not appropriate to consider safety when introducing robots into a workplace?
%NEVER.
%In terms of safety, what are some benefits of collaborative robots?
%Allows you to minimise the amount of investment required and peripheral safety devices such as light curtains and cages. Allows you to maximise the production workspace and workforce.

%% Sawyer
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc


%% Max relative velocity
close all
clear
clc
clf

steps = 50;

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6];
q2 = [-pi/10, -pi/7, -pi/5, -pi/3, -pi/4, -pi/6];

s = lspb(0,1,steps);                                             	% First, create the scalar function
        qMatrix = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
            
            velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

max(abs(velocity))