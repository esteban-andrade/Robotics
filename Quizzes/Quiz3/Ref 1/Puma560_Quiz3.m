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
goalPosition = [0.6 -0.1 -0.2];

% Goal orientation as [roll pitch yaw] in degrees
goalOrientation = [0 0 0];

goalPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');

if useClosedFormSolution
    goalQ = p560.ikine6s(goalPose)
else
    % Mask vector
    mask = [1 1 1 0 0 0];
    q = qn
    goalQ = p560.ikine(goalPose, q, mask)
end

p560.animate(goalQ)
drawnow();

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