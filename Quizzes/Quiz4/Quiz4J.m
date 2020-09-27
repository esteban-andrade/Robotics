%% Puma 560
clear
clf
mdl_puma560
robot = p560
robot.teach();

changeBase = false;
% New base position as [x y z]
basePosition = [0 0 0];

% New base orientation as [roll pitch yaw] in degrees
baseOrientation = [0 0 0];

if changeBase
    robot.base = transl(basePosition) * rpy2tr(baseOrientation, 'deg');
end

%% Baxter
clear
clf
mdl_baxter

LEFT = 0;
RIGHT = 1;

% Set this as either LEFT or RIGHT
robotToUse = LEFT;

if robotToUse == LEFT
    robot = left;
end

if robotToUse == RIGHT
    robot = right;
end

robot
robot.teach()

%% 5 DOF Planar Robot
clear
clf
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

%% Manipulability Measure
% q = robot.getpos();

% q = [0 1.5708 -3.0159 0.1466 0.5585 0]
% q = [0 2.3562 -3.0159 0 -0.9076 0]
% q = [1.1170 1.0996 -3.4872 0.1466 0.5585 0.6500]
q = [0 0.7 3 0 0.7 0]

robot.plot(q)

jn = robot.jacobn(q)
j0 = robot.jacob0(q)

inv(j0)

robot.vellipse(q);
robot.maniplty(q, 'yoshikawa')


limit = robot.islimit(q)
%% Collision Checking - Rectangular Prism (using triangle and line method)
hold on
% centerpnt = [1,0,-0.5];
% side = 1.5;
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
[vertex,faces,faceNormals] = RectangularPrism([2,-1.1,-1], [3,1.1,1]);
axis equal
camlight

% Get the transform of every joint (i.e. start and end of every link)
q = robot.getpos();
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
            i
            intersectP;
        end
    end    
end

%% Collision Detection Points
hold on
cubeLength = 0.5;
cubeCentre = [1,0,-0.5];
sideArray = linspace(-cubeLength / 2, cubeLength / 2, 50);

[Y,Z] = meshgrid(sideArray, sideArray);
gridSize = size(Y);
X = repmat(cubeLength / 2, gridSize(1), gridSize(2));

cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];

% Plot the cube's point cloud         
cubePoints = cubePoints + repmat(cubeCentre,size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');

centrePoint = [0,0,0];

for i = 1:robot.n+1
    linkLength = 1
    if linkLength > 0
        radii = [linkLength,linkLength/2,linkLength/2];
        [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(1), radii(2), radii(3) );
        robot.points{i} = [X(:),Y(:),Z(:)];
        warning off
        robot.faces{i} = delaunay(robot.points{i});
        warning on;
    else
        robot.points{i} = [0, 0, 0];
        robot.faces{i} = [0, 0, 0, 0];
    end
end
robot.plot3d(robot.getpos());
drawnow()
axis equal

linkTransforms = GetJointTransforms(robot, robot.getpos());
count = 0;
for i = 1:size(linkTransforms, 3)
    centrePose = linkTransforms(:,:,i);
    newPoints = [inv(centrePose) * [cubePoints, ones(size(cubePoints,1),1)]']';
    xd = (newPoints(:,1) - centrePoint(1)) / radii(1);
    yd = (newPoints(:,2) - centrePoint(2)) / radii(2);
    zd = (newPoints(:,3) - centrePoint(3)) / radii(3);
    algebraicDistance = xd.^2 + yd.^2 + zd.^2;

    newPoints(:,4) = algebraicDistance < 1;

    for i = 1:size(newPoints, 1)
        if newPoints(i,4) == 1
            count = count + 1;
            plot3(cubePoints(i,1), cubePoints(i,2), cubePoints(i,3), 'Color', 'r', 'LineStyle', 'none', 'Marker', '*')
        end
    end
end
count