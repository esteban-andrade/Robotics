%% Lab 6 My solutions
close all
clear all
clc

set(0,'DefaultFigureWindowStyle','docked')
view (3)

% Consider a laser tape measure (LTM)1 that projects a laser and measures a single distance to
% an object in the environment Pretend it is affixed to the end effector of the Schunk robot
  
robot = SchunkUTSv2_0();
q = [0,pi/2,0,0,0,0];
robot.plot3d(q);
view(3);
camlight;
hold on;

%% Assume the LTM (max range 3m) has an identical start location as the end-effector location, and the ray is
% parallel to the Z axis. Plot a red line 1.9594m long from end effector parallel with the Z axis

tr = robot.fkine(q);
startP = tr(1:3,4)';
endP = tr(1:3,4)' + 1.9594 * tr(1:3,3)';
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
verts = endP;
axis image


%% If the robot is in a pose [0,pi/2,0,0,0,0] and the laser tape measure returns a measurement (as plotted above),
% the robot is robot is moved twice more and a measurement is taken. Plot this information

q1 = [ pi/10, pi/2, 0, 0, 0, 0 ];
distance1 =  2.4861;
tr=robot.fkine(q1);
robot.animate(q1)
startP = tr(1:3,4)';
endP = tr(1:3,4)' + distance1 * tr(1:3,3)';
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
verts = [verts; endP];
pause (1)

%%
q2 = [ -pi/10, 5*pi/12, 0, 0, 0, 0 ];
distance1 =  1.9132;
tr=robot.fkine(q2);
robot.animate(q2)
startP = tr(1:3,4)';
endP = tr(1:3,4)' + distance1 * tr(1:3,3)';
line1_h = plot3([startP(1),endP(1)],[startP(2),endP(2)],[startP(3),endP(3)],'r');
plot3(endP(1),endP(2),endP(3),'r*');
verts = [verts; endP];
pause (1)

%% Given now these three points, make a mesh of the possible location of a wall (only use value of the wall which
% are above 0m). Remember, where the triangle vertices are [v1,v2,v3]
% triangleNormal = unit(cross((v1-v2),(v2-v3)))
triangleNormal = unit(cross(verts(1,:)-verts(2,:),verts(2,:)-verts(3,:)))

%make a plane at origin
basePlaneNormal = [-1,0,0];
[Y,Z] = meshgrid(-2:0.1:2,-2:0.1:2  );
sizeMat = size(Y);
X = zeros(sizeMat(1),sizeMat(2));

% Rotation axis: to rotate the base plane around
rotationAxis = cross(triangleNormal,basePlaneNormal);
rotationAxis = rotationAxis / norm(rotationAxis);

% Rotation angle: how much to rotate base plane to make it match triangle plane
rotationRadians = acos(dot(triangleNormal,basePlaneNormal));

% Make a transform to do that rotation
tr = makehgtform('axisrotate',rotationAxis,rotationRadians);

% Find a central point of the triangle
trianglePoint = sum(verts)/3;

% Plot the point/normal of the triangle
plot3(trianglePoint(1),trianglePoint(2),trianglePoint(3),'g*');
plot3([trianglePoint(1),trianglePoint(1)+triangleNormal(1)] ...
     ,[trianglePoint(2),trianglePoint(2)+triangleNormal(2)] ...
     ,[trianglePoint(3),trianglePoint(3)+triangleNormal(3)],'b');
 drawnow();
 pause(1);
 
% Transform the points on the default plane, to matches the actual triangle
points = [X(:),Y(:),Z(:)] * tr(1:3,1:3) + repmat(trianglePoint,prod(sizeMat),1);
X = reshape(points(:,1),sizeMat(1),sizeMat(2));
Y = reshape(points(:,2),sizeMat(1),sizeMat(2));
Z = reshape(points(:,3),sizeMat(1),sizeMat(2));

% Make points where Z<0 to be = zero
Z(Z<0)= 0;
surf(X,Y,Z);
pause(1);

%% Consider mounting a motor to rotate (roll) the LTM 40’ around the X
%axis of the end effector, and look at the wall. Put the robot in a pose
%[0,pi/2,0,0,0,0] and rotate the LTM by increments of 1’ from -20’ to 20’
%(total of 41 readings). Essentially this is a Laser Range Finder (LRF)2.
maxRange = 3; % meters

% get end-effector point in given pose
q = [0,pi/2,0,0,0,0];
tr = robot.fkine(q);
robot.animate(q);
startP = tr(1:3,4)';

% Make a single scan ray as if it were from the origin (for rotating)
rayAtOrigin = maxRange * tr(1:3,3)';
xRotAxis = tr(1:3,1)';
yRotAxis = tr(1:3,2)';

% Rotate around end effector's xaxis and yaxis, populate scanData
scanData = [];
for xRotRads = deg2rad(-20):deg2rad(1):deg2rad(20)
    for yRotRads = deg2rad(-20):deg2rad(1):deg2rad(20) % Note it is more efficient to make one scan block rather than having an embeded for loop as done here
        % Make a transform to rotate the scan ray around
        tr = makehgtform('axisrotate',xRotAxis,xRotRads) * makehgtform('axisrotate',yRotAxis,yRotRads);
        
        % Determine location of ray end at max range
        rayEnd = startP +  rayAtOrigin * tr(1:3,1:3);
        
        % Check for intersection with the plane that the triangle is on
        [intersectP,check] = LinePlaneIntersection(triangleNormal,trianglePoint,startP,rayEnd);        
        if check == 1
            rayEnd = intersectP;
        end
        
        % Check for intersection with floor (note: floorNormal = [0,0,1] floorPoint = origin)
        [intersectWithFloor,check] = LinePlaneIntersection([0,0,1],[0,0,0],startP,rayEnd);
        if check == 1 && dist2pts(startP,intersectWithFloor) < dist2pts(startP,rayEnd)
            rayEnd = intersectWithFloor;
        end
        
        % Keep track of the scan data. It is more efficient to initialise
        % it to the correct size, however for brevity, it is done suboptimally here
        scanData = [scanData; rayEnd];         %#ok<AGROW>
    end
end
% Plot the scan data
plot3(scanData(:,1),scanData(:,2),scanData(:,3),'r.');
