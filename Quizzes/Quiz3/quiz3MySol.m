%% quiz 3 my solution
close all
set(0,'DefaultFigureWindowStyle','docked')
clear all
clc
view(3)
% Create a 3 link 3D robot  when q =... robot call r3 collide with a flat
% wall at x. use linePlane Intersection
mdl_3link3d;
q = [pi/12,0,0];
endEffectorTR = R3.fkine(q);
R3.teach(q);
hold on;
planeXntersect = 3;
planeBounds = [planeXntersect-eps,planeXntersect+eps,-3,3,-3,3]; 
[Y,Z] = meshgrid(planeBounds(3):0.1:planeBounds(4),planeBounds(5):0.1:planeBounds(6));
X = repmat(planeXntersect,size(Y,1),size(Y,2));
surf(X,Y,Z);
point1OnLine = endEffectorTR(1:3,4)';
point2OnLine = [0 0 1];
normal = [-3 0 0]
point = [3 0 0]
[intersectionPoint,check] = LinePlaneIntersection(normal,point,point1OnLine,point2OnLine)

%% Create a puma 560 Assume is on the flor with q. have a ball on centre at. Where is the ball position with respect to end effector.

clc 
clear all 
close all
set(0,'DefaultFigureWindowStyle','docked')
view (3)

mdl_puma560;

q = deg2rad([0, 45, -80, 0, 45, 0]);
endPose = p560.fkine(q);
p560.teach(q);
hold on

ball=  transl(0.4,-0.2,0.7) * trotx(-pi/2) ;
trplot(ball);

DistanceEndEffectorToBall = inv(endPose)*ball;
FinalDist = DistanceEndEffectorToBall(1:3,4)'

relativePose = endPose\ball;

relativePosition = transl(relativePose)'

%% Which of the following safety measures could be used to reduce the damage
% a robot could do to a person who need to work in close proximity with a
% robot

clc
disp('Physical Barriers')

%%
clc
%When is it not appropriate to consider safety when introducing robots into a workplace?
disp('NEVER');

%% 
clc
%%In terms of safety, what are some benefits of collaborative robots?
 disp('Allows you to minimise the amount of investment required and peripheral safety devices such as light curtains and cages. Allows you to maximise the production workspace and workforce.');
%%
clc
% collaboratives robots such as the sawyer are more expensice since they
% are designed to work in close proximity with humans. Do Collaborative
% robots alwys require additional safety infraestructure to be intalled.

disp('it depends on the application')
%%
clc
% In terms of efficiency motor movement what is the best way to interpolate
% between two joint states 
disp ('Trapezoidal Velocity profile')

%%
clc
 % what is the concern with introducing rbbots such as SAWYER into
 % production lines
 disp('People are concern about the robot taking low wage jobs');
 
 %%
 clc
 % which of the following safety measures could be used to reduce the
 % damage a robot could do to a person who needs to work in close proximity
 % with a robot
 disp('reduce robot speed');
 
 %% 
 clc
 % does ikine consder joint angles limit 
 
 disp('No unless is masked');
 
 %% 
 clc
 
 %what was the previous robot to the sawyer (2 arms)
 disp('BAxter')
 
 %% create a puma 560 distance sensor 
 clc 
 clear all 
 close all 
 set(0,'DefaultFigureWindowStyle','docked')
view (3)

mdl_puma560;
p560.base = transl(0,0,0);
q = deg2rad([0, 45, -80, 0, 45, 0]); % Q INPUT
endEffectorTr = p560.fkine(q);
X = 1
Y = -2.276
Z = 1

distanceSensorTr = transl(X,Y,Z)
% %p560.teach(q)
dist = sqrt((distanceSensorTr(1,4) - endEffectorTr(1,4))^2 + ...
    (distanceSensorTr(2,4) - endEffectorTr(2,4)^2) + ...
    (distanceSensorTr(3,4) - endEffectorTr(3,4))^2)
y = sqrt(2^2-(1-endEffectorTr(1,4))^2-(1-endEffectorTr(3,4))^2)+endEffectorTr(2,4)


endEffector = p560.fkine(q);
endEffectorPosition = transl(endEffector)'

objectPosition = [-0.2165, -0.1500, 1.0620]


hold on
trplot(transl(objectPosition));

diffPosition = endEffectorPosition - objectPosition;
distance = sqrt(sum(diffPosition.^2))


%sensors = transl(1,2,1);
%pose = inv(sensors)*endEffector

%% Create puma 560. When q is at ... determine a ray cast from z axis with normal and pint. 



close all
set(0,'DefaultFigureWindowStyle','docked')
view(3)
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



endEffector = p560.fkine(q);
endEffectorPosition = transl(endEffector)';

lineSegmentOffset = [0 0 1];
lineSegmentEndPoint = transl(endEffector * transl(lineSegmentOffset))';

hold on
points = [endEffectorPosition; lineSegmentEndPoint]
plot3(points(:,1), points(:,2), points(:,3), 'Color', 'r', 'LineStyle', '-', 'Marker', '+')

% Create a surface plane that goes through [x y z] with a normal vector [A B C]
planePoint = point;
planeNormal = normal;

A = planeNormal(1);
B = planeNormal(2);
C = planeNormal(3);
D = sum(planePoint .* planeNormal);

v = -2:0.1:2;
if A ~= 0
    [y, z] = meshgrid(v, v);
    x = -1/A*(B*y + C*z - D);
end

if B ~= 0
    [x, z] = meshgrid(v, v);
    y = -1/B*(A*x + C*z - D);
end

if C ~= 0
    [x, y] = meshgrid(v, v);
    z = -1/C*(A*x + B*y - D);
end

hold on;
surf(x,y,z);

[intersectionPoint,check] = LinePlaneIntersection(planeNormal,planePoint,endEffectorPosition,lineSegmentEndPoint)
if (check == 1) || (check == 3)
    intersectionPointPlot_h = plot3(intersectionPoint(:,1),intersectionPoint(:,2),intersectionPoint(:,3),'g*');
    
    % Find distance from j1 to j2
    diffPosition = endEffectorPosition - intersectionPoint;
    distance = sqrt(sum(diffPosition.^2))
end
%% Given 2 joint states q1 q2. create a 50 step trayectory with trapezoudal velocity profile


close all
clear
clc
clf


steps = 45;

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6];
q2 = [-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10];

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

max_array= max(abs(velocity))
maxVel = max(max_array)

%% CReate a puma 560 use ikine to determine a joint state such positino end effector.  mask it off

close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc
% Create Puma
mdl_puma560;

%End effector pos= [0.6 -0.1 -0.2]
endEffectorTr = transl(0.6,-0.1,-0.2)

q = p560.ikine(endEffectorTr,qn, [1 1 1 0 0 0])   

%% 5 Dof Plannar 

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
       
q = deg2rad([30,-30,30,-30,0]);
%qzero = zeros(1,5)
endEffectorTr = fiveDOF.fkine(q)
% fiveDOF.teach(q)

endEffectorPosition  = endEffectorTr(1:3,4)'














%%


%% Puma 560
clear
mdl_puma560
p560
p560.teach();

changeBase = false;
% New base position as [x y z]
basePosition = [0 0 0];

% New base orientation as [roll pitch yaw] in degrees
baseOrientation = [0 0 0];

if changeBase
    p560.base = transl(basePosition) * rpy2tr(baseOrientation, 'deg');
end




%%
% Distance to wall along Z

% Joint configuration
q = [pi/8,0,-pi/2,0,0,0];

endEffector = p560.fkine(q);
endEffectorPosition = transl(endEffector)';

lineSegmentOffset = [0 0 1];
lineSegmentEndPoint = transl(endEffector * transl(lineSegmentOffset))';

hold on
points = [endEffectorPosition; lineSegmentEndPoint]
plot3(points(:,1), points(:,2), points(:,3), 'Color', 'r', 'LineStyle', '-', 'Marker', '+')

% Create a surface plane that goes through [x y z] with a normal vector [A B C]
planePoint = [2.9 0 0];
planeNormal = [-1 0 0];

A = planeNormal(1);
B = planeNormal(2);
C = planeNormal(3);
D = sum(planePoint .* planeNormal);

v = -2:0.1:2;
if A ~= 0
    [y, z] = meshgrid(v, v);
    x = -1/A*(B*y + C*z - D);
end

if B ~= 0
    [x, z] = meshgrid(v, v);
    y = -1/B*(A*x + C*z - D);
end

if C ~= 0
    [x, y] = meshgrid(v, v);
    z = -1/C*(A*x + B*y - D);
end

hold on;
surf(x,y,z);

[intersectionPoint,check] = LinePlaneIntersection(planeNormal,planePoint,endEffectorPosition,lineSegmentEndPoint)
if (check == 1) || (check == 3)
    intersectionPointPlot_h = plot3(intersectionPoint(:,1),intersectionPoint(:,2),intersectionPoint(:,3),'g*');
    
    % Find distance from j1 to j2
    diffPosition = endEffectorPosition - intersectionPoint;
    distance = sqrt(sum(diffPosition.^2))
end





%% Point in the end effector’s coordinate frame

% Point position as [x y z]
goalPosition = [0.4 -0.2 0.7];

% Point orientation as [roll pitch yaw] in degrees
goalOrientation = [-90 0 0];

% Joint configuration
q = [90, 30, -80, 0, 45, 0];

endEffector = p560.fkine(q, 'deg');
newPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');

relativePose = endEffector \ newPose;

relativePosition = transl(relativePose)


%% Straightforward Inverse kinematics using robotics toolbox
useClosedFormSolution = false;

% Goal position as [x y z]
goalPosition = [0.6,0.1,0.1];

% Goal orientation as [roll pitch yaw] in degrees
goalOrientation = [0 0 0];

goalPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');

if useClosedFormSolution
    goalQ = p560.ikine6s(goalPose)
else
    % Mask vector
    mask = [1 1 1 0 0 0];
    q = qn;
    goalQ = p560.ikine(goalPose, q, mask)
end

%p560.animate(goalQ)
%drawnow();



%% Distance sense distance to Puma End Effector
% Joint configuration
q = [0, 40, -80, 0, 45, 0];
q = deg2rad(q);
% q = p560.getpos();
endEffector = p560.fkine(q);
endEffectorPosition = transl(endEffector)'

objectPosition = [1, 1.9794, 1];
hold on
trplot(transl(objectPosition));

diffPosition = endEffectorPosition - objectPosition;
distance = sqrt(sum(diffPosition.^2))


%% Collision Checking
% Make a 3DOF model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);       
robot = SerialLink([L1 L2 L3],'name','myRobot');                     
q = zeros(1,3);    
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];
robot.plot(q,'workspace',workspace,'scale',scale);

centerpnt = [2,0,-0.5];
side = 1.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
camlight
robot.teach;

% Get the transform of every joint (i.e. start and end of every link)
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end

% Go through until there are no step sizes larger than 1 degree
q1 = [-pi/4,0,0];
q2 = [pi/4,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end

%% 5 DOF Planar Robot
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);      
L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);  
L5 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);  
robot = SerialLink([L1 L2 L3 L4 L5],'name','robot5dof');                     
q = zeros(1,5);
robot.plot(q);
robot.teach;
xlim([-6 6]);
ylim([-6 6]);
zlim([-6 6]);

%%
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

steps = 45;

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6];
q2 = [-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10];

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

max_array= max(abs(velocity))
maxVel = max(max_array)
