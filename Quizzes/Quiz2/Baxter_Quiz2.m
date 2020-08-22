%% Baxter
clear
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

%%
% Distance between arms in certain joint states
% Distance between bases and/or arms in certain joint states
baseIndex = 1;

j1 = baseIndex;
j2 = 3;

q = robot.getpos()
jointTransforms = GetJointTransforms(robot, q);
jointPositions = zeros(robot.n + 1, 3);
for i = 1:(robot.n + 1)
    jointPositions(i,:) = transl(jointTransforms(:,:,i))';
end
jointPositions
hold on
plot3(jointPositions(:,1), jointPositions(:,2), jointPositions(:,3), 'Color', 'k', 'LineStyle', '-', 'Marker', '+')
drawnow();

pos1 = jointPositions(j1,:)
pos2 = jointPositions(j2,:)

% Find distance from j1 to j2
diffPosition = pos1 - pos2;
distance = sqrt(sum(diffPosition.^2))

%% Straightforward Inverse kinematics using robotics toolbox
% Goal position as [x y z]
goalPosition = [0.1 0.2 0.3];

% Goal orientation as [roll pitch yaw] in degrees
goalOrientation = [0 0 0];

% Mask vector
mask = [1 1 1 0 0 0];

goalPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');
goalQ = robot.ikine(goalPose, robot.getpos(), mask)

robot.animate(goalQ)

