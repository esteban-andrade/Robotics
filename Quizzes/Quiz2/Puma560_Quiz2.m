%% Puma 560
clear
mdl_puma560
p560
p560.teach();

changeBase = true;
% New base position as [x y z]
basePosition = [0 0 0.8];

% New base orientation as [roll pitch yaw] in degrees
baseOrientation = [0 0 0];

if changeBase
    p560.base = transl(basePosition) * rpy2tr(baseOrientation, 'deg');
end

%%
% Distance from end effector to surface in environment
% Distance from end effector to surface in environment given new information about base location

% Joint configuration
q = p560.getpos();

endEffector = p560.fkine(q);
endEffectorPosition = transl(endEffector)';

lineSegmentOffset = [0 0 1];
lineSegmentEndPoint = transl(endEffector * transl(lineSegmentOffset))';

hold on
points = [endEffectorPosition; lineSegmentEndPoint]
plot3(points(:,1), points(:,2), points(:,3), 'Color', 'r', 'LineStyle', '-', 'Marker', '+')

% Create a surface plane that goes through [x y z] with a normal vector [A B C]
planePoint = [0.8 0 0];
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
goalPosition = [0.5 0 0.6];

% Point orientation as [roll pitch yaw] in degrees
goalOrientation = [90 0 0];

% Joint configuration
q = [0, 45, 45, 0, 45, 0];

endEffector = p560.fkine(q, 'deg');
newPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');

relativePose = endEffector \ newPose;

relativePosition = transl(relativePose)

%% Straightforward Jacobian analysis p.171
q = p560.getpos();
jn = p560.jacobn(q)
j0 = p560.jacob0(q)

inv(j0)

p560.vellipse(q);

%% Straightforward Inverse kinematics using robotics toolbox
useClosedFormSolution = true;

% Goal position as [x y z]
goalPosition = [0.1 0.2 0.3];

% Goal orientation as [roll pitch yaw] in degrees
goalOrientation = [0 0 0];

goalPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');

if useClosedFormSolution
    goalQ = p560.ikine6s(goalPose)
else
    % Mask vector
    mask = [1 1 1 1 1 1];
    q = p560.getpos();
    goalQ = p560.ikine(goalPose, q, mask)
end

p560.animate(goalQ)
drawnow();

