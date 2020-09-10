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
%%

% question 2 More complex collision detection for 3-link planar robot
clf
clear all
clc
% Create vertices that represent an ellipsoid with radii (rx=3,ry=2,rz=1) centered at [xc,yc,zc] = [0,0,0].

alpha(0.1) % makes  the elipsoid translucid

centerPoint = [0,0,0];
radii = [3,2,1];
[X,Y,Z] = ellipsoid(centerPoint (1),centerPoint (2),centerPoint(3),radii(1),radii(2),radii(3));
%plot it
view(3)
axis equal
hold on
elipsoidAtOrigin = surf(X,Y,Z);

%% Put a cube with sides 1.5m in the environment that is centered at [2,0,-0.5]. Use mesh so as to create a high density mesh that has many vertices (either create in blender and load, or use 6 planes from meshgrid). Note: create a single plane of a cube centered at the origin like follows:
% One side of the cube
alpha(0.1)
[Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         
         
%% Plot the cube's point cloud         
cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal

%% 2.4 Check how many point and which points are inside the ellipsoid, using the equation. Note that points that are inside have an algebraic distance (AD) < 1, on the surface AD = 1 and outside AD > 1

algebraicDistance = GetAlgebraicDist(cubePoints,centerPoint,radii);
pointsInside = find(algebraicDistance<1);
disp(['There are ',num2str(size(pointsInside,1)),' points inside']);

%% 2.5 Transform the ellipsoid by translating it [1,1,1], do this by changing the values of [xc,yc,zx], then check which points are inside the ellipsoid
centerPoint=[1,1,1];
algebraicDistance = GetAlgebraicDist(cubePoints,centerPoint,radii);
pointsInside = find(algebraicDistance<1);
disp(['There are ',num2str(size(pointsInside,1)),' points inside After motinv the centre to ',num2str(centerPoint)]);

%% 2.6 This time, using the original centered-at-the-origin ellipse, notice how you can transform the points in the environment by inv(transl(1,1,1)) and then check the original equation to see which have an algebraic distance less than 0. The points inside should be the same as when using the previous method.
centerPoint=[0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1)) * [cubePoints,ones(size(cubePoints,1),1)]']'; %#ok<MINV>
updatedCubePoints = cubePointsAndOnes(:,1:3) ;
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.6: There are now ', num2str(size(pointsInside,1)),' points inside']);
%% 2.7 Now, if the ellipsoid where transformed by transl(1,1,1)*trotx(pi/4), which points are inside (note that you will need to transform the points in the environment instead of the ellipsoid formula
centerPoint=[0,0,0];
cubePointsAndOnes = [inv(transl(1,1,1))*trotx(pi/4) * [cubePoints,ones(size(cubePoints,1),1)]']'; %#ok<MINV>
updatedCubePoints = cubePointsAndOnes(:,1:3) ;
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.7: There are now ', num2str(size(pointsInside,1)),' points inside']);


%% 2.8 Now create a 3 link planar and use the 3 ellipsoids as the model points and faces. Now use teach to move it around so you should see the ellipsoids move around as well
try delete(cubeAtOigin_h); end;
try delete(elipsoidAtOrigin); end;
try delete(oneSideOfCube_h); end;


L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])        
robot = SerialLink([L1 L2 L3],'name','myRobot');  
centerPoint = [0,0,0];
radii = [1,0.5,0.5];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:4
    robot.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robot.faces{i} = delaunay(robot.points{i});    
    warning on;
end

robot.plot3d([0,0,0]);
axis equal
camlight

%% 2.9 (Bonus) For a given pose, work out the location of the ellipsoid of the end effector, using fkine, and the multiply the points in the environment by the inverse of this transform, and check the algebraic distance
q = [0,0,0];
tr = robot.fkine(q);
cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
updatedCubePoints = cubePointsAndOnes(:,1:3);
algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
pointsInside = find(algebraicDist < 1);
display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);

%% 2.10 (Bonus) Do this for each of the ellipsoids on the three links. Note: you will need to have your own forward kinematics routine so you can compute the location of each of the ellipsoids
q = [0,0,0];
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each ellipsoid
for i = 1: size(tr,3)
    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
    updatedCubePoints = cubePointsAndOnes(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 1);
    display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
end

%% 3 Joint Interpolation vs Resolve Motion Rate Control
% 3.1 Moving from A to B with Joint Interpolation: Load a 2-Link Planar Robot with mdl_planar2
clf
clear all
clc
steps = 50;
mdl_planar2;  

%% 3.2 Create two Transformation Matrices
T1 = [eye(3) [1.5 1 0]'; zeros(1,3) 1];       % First pose
T2 = [eye(3) [1.5 -1 0]'; zeros(1,3) 1];      % Second pose
%% 3.3 Use Inverse Kinematics to solve the joint angles required to achieve each pose.
M = [1 1 zeros(1,4)]; % Masking Matrix
q1 = p2.ikine(T1,[0 0],M); % Solve for joint angles
q2 = p2.ikine(T2,[0 0],M); % Solve for joint angles
p2.plot(q1,'trail','r-');

%% 3.4 Use joint interpolation to move between the two poses. Be sure to plot the end-effector path.
qmatrix = jtraj(q1,q2,steps);
p2.plot(qmatrix,'trail','r-');

%% 3.6 Create two sets of points in the X-Y plane
x1 = [1.5 1]';
x2 = [1.5 -1]';
deltaT = 0.01;

%%   3.7 Create a matrix of waypoints
x = zeros(2,steps); % Assign memory
s = lspb(0,1,steps); % Create interpolation scalar
for i = 1:steps
x(:,i) = x1*(1-s(i)) + s(i)*x2; % Interpolate waypoints
end

%% 3.8 Create a matrix of joint angles
qMatrix = nan(steps,2);

%% 3.9 Set the Transformation for the 1st point, and solve for the joint angles
qMatrix(1,:) = p2.ikine(T1,[0 0],M);
%% 3.10 Use Resolved Motion Rate Control to move the end-effector from x1 to x2.
for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
    J = J(1:2,:);                           % Take only first 2 rows
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
end

p2.plot(qMatrix,'trail','r-');