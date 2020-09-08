%% LAB 5 my Solutions

clc
clf
clear all 
close all 
%% Create and plot a UFO Fleet of 10 ships
ufoFleet = UFOFleet(10);

%% Create the blaster robot. Note this is the actual “Grit Blasting”1 robot working on the Sydney Harbor Bridge
blasterRobot = SchunkUTSv2_0();
plot3d(blasterRobot,zeros(1,6));
endEffectorTr = blasterRobot.fkine(zeros(1,6));


blasterRobot.delay = 0;

%% Now plot a “blast” cone coming out the end effector (assume it is the Z axis of the end effector.
[X,Y,Z] = cylinder([0,0.1],6);
Z = Z * 10;
updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';
conePointsSize = size(X);
cone_h = surf(reshape(updatedConePoints(:,1),conePointsSize) ...
             ,reshape(updatedConePoints(:,2),conePointsSize) ...
             ,reshape(updatedConePoints(:,3),conePointsSize));
view(3);

%% Now plot a “score board”
currentScore = 0;
scoreZ = ufoFleet.workspaceDimensions(end)*1.2;
text_h = text(0, 0, scoreZ,sprintf('Score: 0 after 0 seconds'), 'FontSize', 10, 'Color', [.6 .2 .6]);

%% Add the following “while loop” to iteratively call your function
% Start timer
tic

% Go through iterations of randomly move UFOs, then move robot. Check for
% hits and update score and timer
while ~isempty(find(0 < ufoFleet.healthRemaining,1))
    ufoFleet.PlotSingleRandomStep();    
    
    % Get the goal joint state
    goalJointState = GetGoalJointState(blasterRobot,ufoFleet,3);

    % Fix goal pose back to a small step away from the min/max joint limits
    fixIndexMin = goalJointState' < blasterRobot.qlim(:,1);
    goalJointState(fixIndexMin) = blasterRobot.qlim(fixIndexMin,1) + 10*pi/180;
    fixIndexMax = blasterRobot.qlim(:,2) < goalJointState';
    goalJointState(fixIndexMax) = blasterRobot.qlim(fixIndexMax,2) - 10*pi/180;
        
    % Get a trajectory
    jointTrajectory = jtraj(blasterRobot.getpos(),goalJointState,8);
    for armMoveIndex = 1:size(jointTrajectory,1)
        animate(blasterRobot,jointTrajectory(armMoveIndex,:));
         
        endEffectorTr = blasterRobot.fkine(jointTrajectory(armMoveIndex,:));        
        updatedConePoints = [endEffectorTr * [X(:),Y(:),Z(:),ones(numel(X),1)]']';        
        set(cone_h,'XData',reshape(updatedConePoints(:,1),conePointsSize) ...
                  ,'YData',reshape(updatedConePoints(:,2),conePointsSize) ...
                  ,'ZData',reshape(updatedConePoints(:,3),conePointsSize));
              
        coneEnds = [cone_h.XData(2,:)', cone_h.YData(2,:)', cone_h.ZData(2,:)'];        
        ufoHitIndex = CheckIntersections(endEffectorTr,coneEnds,ufoFleet);
        ufoFleet.SetHit(ufoHitIndex);
        currentScore = currentScore + length(ufoHitIndex);
        
        text_h.String = sprintf(['Score: ',num2str(currentScore),' after ',num2str(toc),' seconds']);
        axis([-6,6,-6,6,0,10]);
        % Only plot every 3rd to make it faster
        if mod(armMoveIndex,3) == 0
            drawnow();
        end
    end
    drawnow();
end
display(['Finished in ',num2str(toc),' seconds with score of ',num2str(currentScore)]);
% Log results to file

%% Question 2 Simple collision checking for 3-link planar robot
clf
clear all 
close all 
clc
%%
% Create a 3 link planar robot with all 3 links having a = 1m, leave the base at eye(4).
set(0,'DefaultFigureWindowStyle','docked')
view (3)
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);  
robot = SerialLink([L1 L2 L3],'name','myRobot');                     
q = zeros(1,3);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);  

%% Put a cube with sides 1.5m in the environment that is centered at [2,0,-0.5].
% 
centrePoint = [2,0,-0.5];
side = 1.5;
plotOptions.plotFaces = true;
[vertex,face,faceNormals] = RectangularPrism(centrePoint-side/2,centrePoint+side/2,plotOptions);
axis equal;
camlight

% pPoints = [1.25,0,-0.5 ...
%         ;2,0.75,-0.5 ...
%         ;2,-0.75,-0.5 ...
%         ;2.75,0,-0.5];
% pNormals = [-1,0,0 ...
%             ; 0,1,0 ...
%             ; 0,-1,0 ...
%             ;1,0,0];

robot.teach;
%% Using your understanding of forward kinematics write a function that you can pass in vector of joint angles, q, to represent a joint state of the arm, and it will return a 4x4x4 matrix, TR which contains

tr = zeros(4,4,robot.n+1) % create 4 matrixes each 4 by 4
tr (:,:,1) = robot.base;
L=robot.links; % gets link details

for i=1:robot.n
    tr(:,:,i+1)=tr(:,:,i)*trotz(q(i)+L(i).offset) *transl(0,0,L(i).d)* transl(L(i).a,0,0) * trotx(L(i).alpha);
end

%% Use LinePlaneIntersection.m to check if any of the links (i.e. the link n intersects with any of the 4 planes of the cube when q = [0,0,0]. Note how the link n is a line from position TR(1:3,4,n-1) to TR(1:3,4,n)

%Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(face,1)
        vertOnPlane = vertex(face(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(face(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end
%% Use jtraj(q1,q2,steps) to get a trajectory from q1 = [-pi/4,0,0] to q2 = [pi/4,0,0] and pick a value for steps such that the size of each step is less than 1 degree. Hint: look at the step size in degrees using the following
q1 = [-pi/4,0,0];
q2 = [pi/4,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
%% Check each of the joint states in the trajectory to work out which ones are in collision. Return a logical vector of size steps which contains 0 = no collision (safe) and 1 = yes collision (Unsafe). You may like to use this structure.
result = true(steps,1);
for i =1:steps
    result(i) = IsCollision(robot,qMatrix(i,:),face,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end


%% Basic collision avoidance for 3-link planar robot
% Determine a path from pose q1 = [-pi/4,0,0] degrees to q2 = [pi/4,0,0] that doesn’t collide with the cube from previous question. Use the following 3 methods

robot.animate(q1);
robot.teach;
qWaypoints = [q1,-pi/4,deg2rad([-111,-72]),deg2rad([169,-111,-72]),q2];
if IsCollision(robot,qMatrix,face,vertex,faceNormals)
   disp('Collision'); 
else
    disp('All good');
end
robot.animate(qMatrix);

%% % 3.2: Manually create cartesian waypoints
robot.animate(q1);
qWaypoints = [q1 ; robot.ikcon(transl(1.5,-1,0),q1)];
qWaypoints = [qWaypoints; robot.ikcon(transl(1,-1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,-0.5,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,0,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,0.5,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(1.1,1,0),qWaypoints(end,:))];
qWaypoints = [qWaypoints; robot.ikcon(transl(1.5,1,0),q2)];
qWaypoints = [qWaypoints; q2];
qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
if IsCollision(robot,qMatrix,face,vertex,faceNormals)
   disp('Collision'); 
else
   disp('All good');
end
robot.animate(qMatrix);  

%% % 3.3: Randomly select waypoints (primative RRT)

robot.animate(q1);
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot,qMatrixJoin,face,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            robot.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(robot,qMatrixJoin,face,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,3) - 1) * pi;
            while IsCollision(robot,qRand,face,vertex,faceNormals)
                qRand = (2 * rand(1,3) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
robot.animate(qMatrix)
