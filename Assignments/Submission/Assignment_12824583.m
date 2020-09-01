%% Load Models
close all
clear all %#ok<CLALL>
clc
clf

disp('Esteban Andrade 12824583')
set(0, 'DefaultFigureWindowStyle', 'docked');
view(3);
UR3A = UR3('goku'); %Load UR3. Set a name for UR3
UR5A = LinearUR5(false); %Load Linear UR5. Set gripper to false
axis image %set Axis image

%Set robot Base Transforms
baseATransform = transl(0.65, 0, 0);
baseBTransform = transl(-0.75, -0.35, 0) * trotx(pi/2); %*trotx(pi/2) this is to give it the same  orientation as the UR3;

%Set relative centre between bases
centreA = baseATransform(:, 4);
centreB = baseBTransform(:, 4);
centre = (centreA + centreB) / 2;

%Assign transforms to robot bases
UR3A.model.base = baseATransform;
UR5A.model.base = baseBTransform;

%UR3 & UR5  Joint angles zero state
q = zeros(1, 6);
p = zeros(1, 7);

%Animate Models
animate(UR3A.model, q);
hold on
animate(UR5A.model, p);

%% Get Models Volume and Workspace
%clf

%assign degrees for iteration analysis
degrees = 60;

%determine UR3 parameters.
figure('Name', 'Point Clouds Robot')
UR3A.DrawVolumeArm(degrees);
figure('Name', 'UR3 workspace');
UR3A.getReach();
UR3A.plotArmVolume();
UR3TransveralReach = UR3A.transveralReach;

%display UR3 parameters. Display radius, and volume workspace
disp(['UR3 Maximum Reach in X&Y horizontally: ', num2str(UR3A.transveralReach), ' m']);
disp(['UR3 Maximum Reach in Z vertically: ', num2str(UR3A.verticalReach), ' m']);
disp(['UR3 Calculated Arc radius of workSpace: ', num2str(UR3A.arcRadius), ' m']);
disp(['UR3 Calculated WorkSpace Volume: ', num2str(UR3A.computedVolume), ' m3']);

%Determine UR5 Parameters
UR5Analysis = LinearUR5Analysis;
figure('Name', 'UR5 PointClouds')
pointClouds = UR5Analysis.UR5Volume(UR5A, degrees);
figure('Name', 'UR5 Workspace')
workspaceUR5 = UR5Analysis.plotArmVolume(pointClouds);
UR5TransveralReach = UR5Analysis.getTransveralReach(pointClouds, UR5A);

%display UR5 parameters
disp(['UR5 Maximum Reach in X&Y horizontally: ', num2str(UR5Analysis.getTransveralReach(pointClouds, UR5A)), ' m']);
disp(['UR5 Maximum Reach in Z vertically: ', num2str(UR5Analysis.getVerticalReach(pointClouds, UR5A)), ' m']);
disp(['UR5 Calculated Arc radius of workSpace: ', num2str(UR5Analysis.getArcRadius(pointClouds, UR5A)), ' m']);
disp(['UR5 Calculated WorkSpace Volume: ', num2str(workspaceUR5), ' m3']);

%% Loading Enviroment

disp('Loading Models and Environment');

%Calculate a relative point relative to both robots
middlePointInBetweenRobot = middlePointInBetween(baseATransform, baseBTransform);

%Give relative position and translation for each object to the environment

%create table
tablePosition = [middlePointInBetweenRobot(1) - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) - 0.356 - centre(3)];
tableOrientation = [0, 0, 0];
Table = RecontructObject('table3.ply', tableOrientation, tablePosition, 0);

%create floow
floorPosition = [middlePointInBetweenRobot(1) + 0.7 - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) - centre(3)];
floorOrientation = [0, pi / 2, 0];
Floor = RecontructObjectNonRGB('floor2.ply', floorOrientation, floorPosition);

%BRICK 1 to 4 for UR3. The analysis will be to give 4 bricks relative to
%UR3

brickPosition{1} = [0.5, -0.5, -0.06];
brickOrientation{1} = [pi, 0, 0];
brickDestiny{1} = transl(0, -0.2, 0.06) * rpy2tr(brickOrientation{1});
brick{1} = RecontructObject('Brick.ply', brickOrientation{1}, brickPosition{1}, brickDestiny{1});

brickPosition{2} = [0.5, 0.5, -0.06];
brickOrientation{2} = [pi, 0, 0];
brickDestiny{2} = transl(0, 0.2, 0.06) * rpy2tr(brickOrientation{2});
brick{2} = RecontructObject('Brick.ply', brickOrientation{2}, brickPosition{2}, brickDestiny{2});


brickPosition{3} = [0.75, 0.5, -0.06];
brickOrientation{3} = [pi, 0, 0];
brickDestiny{3} = transl(0, 0, 0.12) * rpy2tr(brickOrientation{3});
brick{3} = RecontructObject('Brick.ply', brickOrientation{3}, brickPosition{3}, brickDestiny{3});


brickPosition{4} = [0.75, -0.5, -0.06];
brickOrientation{4} = [pi, 0, 0];
brickDestiny{4} = transl(0, -0.2, 0.18) * rpy2tr(brickOrientation{4});
brick{4} = RecontructObject('Brick.ply', brickOrientation{4}, brickPosition{4}, brickDestiny{4});


%BRICK 5 to 9 for UR5. The analysis was perform assuming UR5 will move 5
%bricks

brickPosition{5} = [-1, -0.75, -0.06];
brickOrientation{5} = [pi, 0, 0];
brickDestiny{5} = transl(0, 0, 0.06) * rpy2tr(brickOrientation{5});
brick{5} = RecontructObject('Brick.ply', brickOrientation{5}, brickPosition{5}, brickDestiny{5});


brickPosition{6} = [-1, 0.75, -0.06];
brickOrientation{6} = [pi, 0, 0];
brickDestiny{6} = transl(0, 0.2, 0.12) * rpy2tr(brickOrientation{6});
brick{6} = RecontructObject('Brick.ply', brickOrientation{6}, brickPosition{6}, brickDestiny{6});


brickPosition{7} = [-1.5, -0.75, -0.06];
brickOrientation{7} = [pi, 0, 0];
brickDestiny{7} = transl(0, -0.2, 0.12) * rpy2tr(brickOrientation{7});
brick{7} = RecontructObject('Brick.ply', brickOrientation{7}, brickPosition{7}, brickDestiny{7});

brickPosition{8} = [-1.5, 0.75, -0.06];
brickOrientation{8} = [pi, 0, 0];
brickDestiny{8} = transl(0, 0, 0.18) * rpy2tr(brickOrientation{8});
brick{8} = RecontructObject('Brick.ply', brickOrientation{8}, brickPosition{8}, brickDestiny{8});


brickPosition{9} = [-1.5, 0, -0.06];
brickOrientation{9} = [pi, 0, 0];
brickDestiny{9} = transl(0, 0.2, 0.18) * rpy2tr(brickOrientation{9});
brick{9} = RecontructObject('Brick.ply', brickOrientation{9}, brickPosition{9}, brickDestiny{9});

%Create observer
BobPosition = [middlePointInBetweenRobot(1) - centre(1), middlePointInBetweenRobot(2) + 3 - centre(2), middlePointInBetweenRobot(3) - centre(3)];
BobOrientation = [0, 0, 0];
Bob = RecontructObjectNonRGB('full_body.ply', BobOrientation, BobPosition);

%create fencing
fencePosition{1} = [middlePointInBetweenRobot(1) + 2.5 - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) + 0.3 - centre(3)];
fenceOrientation{1} = [0, 0, pi / 2];
fence{1} = RecontructObject('fence.ply', fenceOrientation{1}, fencePosition{1}, 0);

fencePosition{2} = [middlePointInBetweenRobot(1) - 2.5 - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) + 0.3 - centre(3)];
fenceOrientation{2} = [0, 0, pi / 2];
fence{2} = RecontructObject('fence.ply', fenceOrientation{2}, fencePosition{2}, 0);

fencePosition{3} = [middlePointInBetweenRobot(1) + 2.5 - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) + 0.3 - centre(3)];
fenceOrientation{3} = [0, 0, 0];
fence{3} = RecontructObject('fence.ply', fenceOrientation{3}, fencePosition{3}, 0);

fencePosition{4} = [middlePointInBetweenRobot(1) - 2.5 - centre(1), middlePointInBetweenRobot(2) - centre(2), middlePointInBetweenRobot(3) + 0.3 - centre(3)];
fenceOrientation{4} = [0, 0, 0];
fence{4} = RecontructObject('fence.ply', fenceOrientation{4}, fencePosition{4}, 0);

%create emergency stop botton
eButtonPosition = [middlePointInBetweenRobot(1) + 1.7 - centre(1), middlePointInBetweenRobot(2) + 0.7 - centre(2), middlePointInBetweenRobot(3) + 0.1 - centre(3)];
eButtonOrientation = [0, 0, 0];
eButton = RecontructObject('emergency.ply', eButtonOrientation, eButtonPosition, 0);

%display warning signs
warningPosition{1} = [middlePointInBetweenRobot(1) - centre(1), middlePointInBetweenRobot(2) - 2.6 - centre(2), middlePointInBetweenRobot(3) + 0.7 - centre(3)];
warningOrientation{1} = [0, 0, 0];
warningSign{1} = RecontructObject('warning.ply', warningOrientation{1}, warningPosition{1}, 0);

warningPosition{2} = [middlePointInBetweenRobot(1) - centre(1), middlePointInBetweenRobot(2) - 2.6 - centre(2), middlePointInBetweenRobot(3) + 0.7 - centre(3)];
warningOrientation{2} = [0, 0, pi];
warningSign{2} = RecontructObject('warning.ply', warningOrientation{2}, warningPosition{2}, 0);

%animate robot models
animate(UR3A.model, q);
hold on
animate(UR5A.model, p);
disp('Environment Imported');

%% Sort Bricks. Assume and sort the corresponding bricks to each robot

%data structure for each robot allocated bricks
UR3Bricks = [];
UR5Bricks = [];
UnnallocatedBricks = [];

%get number of bricks to each robot
NumberUR3Bricks = 0;
NumberUR5Bricks = 0;

%Iterate for bricks numbers
for i = 1:9

    %Calculate relative distance between bricks and robots
    distUR3 = norm(baseATransform(1:3, 4)-brick{i}.pose(1:3, 4));
    distUR5 = norm(baseBTransform(1:3, 4)-brick{i}.pose(1:3, 4));

    %check distance between bricks and robots and the reach radius for UR3
    %and assign brick to UR#
    if (distUR3 <= distUR5) && (distUR3 <= UR3TransveralReach)
        UR3Bricks = [UR3Bricks, brick{i}];
        NumberUR3Bricks = NumberUR3Bricks + 1;

        % Check if distance of brick will be valid for UR5. Assign valid brick to UR5
    elseif (distUR5 <= UR5TransveralReach)
        UR5Bricks = [UR5Bricks, brick{i}];
        NumberUR5Bricks = NumberUR5Bricks + 1;
    else
        % if not valid assign to unnallocated
        UnnallocatedBricks = [UnnallocatedBricks, brick{i}];
    end
end

%% Kinematics and motion

%give number of steps for interporlation
steps = 60;

% Mode 1 for single arm movement, Mode 2 double arm movement (Order UR3 to UR5).
% Mode 3 will be an automated method that will perform analysis for all
% bricks and motion for each robot. However, This method does not animate
% the bricks motion. Thus only run Method 2.
mode = 2;

%booleans for status
brickCollected = false;
brickDeposited = false;

%Still and original joint configuration.
qUR3still = [0, 0, 0, 0, 0, 0];
qUR5still = [0, 0, 0, 0, 0, 0, 0];

disp('Starting Construction')

%NOTE: After each brick is placed in the final position. The robot will
%return to the still pose. This was added to avoid end effector collision
% for the bricks of walls. Although it will take longer. We will need to
% consider safety of the  equipment


if mode == 1
    %approach brick
    q1_{1} = UR3A.model.getpos();
    [q2_{1}, err2_{1}, exitflag_{1}] = UR3A.model.ikcon(brick{1}.pose, q1_{1});
    qmatrix_{1} = robotMotion.interpolateJointAnglesUR3(q1_{1}, q2_{1}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{1}, UR3A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{1} = qmatrix_{1}(end, :);
        [q4_{1}, err4_{1}, exitflag4_{1}] = UR3A.model.ikcon(brickDestiny{1}, q3_{1});
        qmatrixDes_{1} = robotMotion.interpolateJointAnglesUR3(q3_{1}, q4_{1}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{1}, UR3A.model, brick{1}, brickDestiny{1});

        brickDeposited = true;
        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR3, UR3A.model);
            brickCollected = false; %#ok<*NASGU>
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{2} = UR5A.model.getpos();
    [q2_{2}, err2_{2}, exitflag_{2}] = UR5A.model.ikcon(brick{5}.pose, q1_{2});
    qmatrix_{2} = robotMotion.interpolateJointAnglesLinearUR5(q1_{2}, q2_{2}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{2}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{2} = qmatrix_{2}(end, :);
        [q4_{2}, err4_{2}, exitflag4_{2}] = UR5A.model.ikcon(brickDestiny{5}, q3_{2});
        qmatrixDes_{2} = robotMotion.interpolateJointAnglesLinearUR5(q3_{2}, q4_{2}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{2}, UR5A.model, brick{5}, brickDestiny{5});
        brickDeposited = true;

        if brickDeposited == true
            %return arm to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{3} = UR3A.model.getpos();
    [q2_{3}, err2_{3}, exitflag_{3}] = UR3A.model.ikcon(brick{2}.pose, q1_{3});
    qmatrix_{3} = robotMotion.interpolateJointAnglesUR3(q1_{3}, q2_{3}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{3}, UR3A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{3} = qmatrix_{3}(end, :);
        [q4_{3}, err4_{3}, exitflag4_{3}] = UR3A.model.ikcon(brickDestiny{2}, q3_{3});
        qmatrixDes_{3} = robotMotion.interpolateJointAnglesUR3(q3_{3}, q4_{3}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{3}, UR3A.model, brick{2}, brickDestiny{2});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR3, UR3A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{4} = UR5A.model.getpos();
    [q2_{4}, err2_{4}, exitflag_{4}] = UR5A.model.ikcon(brick{6}.pose, q1_{4});
    qmatrix_{4} = robotMotion.interpolateJointAnglesLinearUR5(q1_{4}, q2_{4}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{4}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{4} = qmatrix_{4}(end, :);
        [q4_{4}, err4_{4}, exitflag4_{4}] = UR5A.model.ikcon(brickDestiny{6}, q3_{4});
        qmatrixDes_{4} = robotMotion.interpolateJointAnglesLinearUR5(q3_{4}, q4_{4}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{4}, UR5A.model, brick{6}, brickDestiny{6});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;

        end
    end

    %approach brick
    q1_{5} = UR3A.model.getpos();
    [q2_{5}, err2_{5}, exitflag_{5}] = UR3A.model.ikcon(brick{3}.pose, q1_{5});
    qmatrix_{5} = robotMotion.interpolateJointAnglesUR3(q1_{5}, q2_{5}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{5}, UR3A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{5} = qmatrix_{5}(end, :);
        [q4_{5}, err4_{5}, exitflag4_{5}] = UR3A.model.ikcon(brickDestiny{3}, q3_{5});
        qmatrixDes_{5} = robotMotion.interpolateJointAnglesUR3(q3_{5}, q4_{5}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{5}, UR3A.model, brick{3}, brickDestiny{3});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR3, UR3A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{6} = UR5A.model.getpos();
    [q2_{6}, err2_{6}, exitflag_{6}] = UR5A.model.ikcon(brick{7}.pose, q1_{6});
    qmatrix_{6} = robotMotion.interpolateJointAnglesLinearUR5(q1_{6}, q2_{6}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{6}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{6} = qmatrix_{6}(end, :);
        [q4_{6}, err4_{6}, exitflag4_{6}] = UR5A.model.ikcon(brickDestiny{7}, q3_{6});
        qmatrixDes_{6} = robotMotion.interpolateJointAnglesLinearUR5(q3_{6}, q4_{6}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{6}, UR5A.model, brick{7}, brickDestiny{7});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;

        end
    end

    %approach brick
    q1_{7} = UR3A.model.getpos();
    [q2_{7}, err2_{7}, exitflag_{7}] = UR3A.model.ikcon(brick{4}.pose, q1_{7});
    qmatrix_{7} = robotMotion.interpolateJointAnglesUR3(q1_{7}, q2_{7}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{7}, UR3A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{7} = qmatrix_{7}(end, :);
        [q4_{7}, err4_{7}, exitflag4_{7}] = UR3A.model.ikcon(brickDestiny{4}, q3_{7});
        qmatrixDes_{7} = robotMotion.interpolateJointAnglesUR3(q3_{7}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{7}, UR3A.model, brick{4}, brickDestiny{4});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR3, UR3A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{8} = UR5A.model.getpos();
    [q2_{8}, err2_{8}, exitflag_{8}] = UR5A.model.ikcon(brick{8}.pose, q1_{8});
    qmatrix_{8} = robotMotion.interpolateJointAnglesLinearUR5(q1_{8}, q2_{8}, steps); %Create Qmatrix based on interpolation
    robotMotion.motion(qmatrix_{8}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{8} = qmatrix_{8}(end, :);
        [q4_{8}, err4_{8}, exitflag4_{8}] = UR5A.model.ikcon(brickDestiny{8}, q3_{8});
        qmatrixDes_{8} = robotMotion.interpolateJointAnglesLinearUR5(q3_{8}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{8}, UR5A.model, brick{8}, brickDestiny{8});
        brickDeposited = true;

        if brickDeposited == true

            %return arm to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{9} = UR5A.model.getpos();
    [q2_{9}, err2_{9}, exitflag_{9}] = UR5A.model.ikcon(brick{9}.pose, q1_{9});
    qmatrix_{9} = robotMotion.interpolateJointAnglesLinearUR5(q1_{9}, q2_{9}, steps);
    robotMotion.motion(qmatrix_{9}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{9} = qmatrix_{9}(end, :);
        [q4_{9}, err4_{9}, exitflag4_{9}] = UR5A.model.ikcon(brickDestiny{9}, q3_{9});
        qmatrixDes_{9} = robotMotion.interpolateJointAnglesLinearUR5(q3_{9}, q4_{9}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{8}, UR5A.model, brick{9}, brickDestiny{9});
        brickDeposited = true;

        if brickDeposited == true
            %return arm to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %display
    disp('Wall Built')
elseif mode == 2

    %approach brick
    q1_{1} = UR3A.model.getpos();
    [q2_{1}, err2_{1}, exitflag_{1}] = UR3A.model.ikcon(brick{1}.pose, q1_{1});
    qmatrix_{1} = robotMotion.interpolateJointAnglesUR3(q1_{1}, q2_{1}, steps); %Create Qmatrix based on interpolation
    %approach brick
    q1_{2} = UR5A.model.getpos();
    [q2_{2}, err2_{2}, exitflag_{2}] = UR5A.model.ikcon(brick{5}.pose, q1_{2});
    qmatrix_{2} = robotMotion.interpolateJointAnglesLinearUR5(q1_{2}, q2_{2}, steps); %Create Qmatrix based on interpolation
    %move arms
    robotMotion.doubleMotion(qmatrix_{1}, UR3A.model, qmatrix_{2}, UR5A.model)
    brickCollected = true;

    if brickCollected == true

        %drop bricks
        q3_{1} = qmatrix_{1}(end, :);
        [q4_{1}, err4_{1}, exitflag4_{1}] = UR3A.model.ikcon(brickDestiny{1}, q3_{1});
        qmatrixDes_{1} = robotMotion.interpolateJointAnglesUR3(q3_{1}, q4_{1}, steps); %Create Qmatrix based on interpolation
        q3_{2} = qmatrix_{2}(end, :);
        [q4_{2}, err4_{2}, exitflag4_{2}] = UR5A.model.ikcon(brickDestiny{5}, q3_{2});
        qmatrixDes_{2} = robotMotion.interpolateJointAnglesLinearUR5(q3_{2}, q4_{2}, steps); %Create Qmatrix based on interpolation
        %move arms
        robotMotion.doubleObjectMotion(qmatrixDes_{1}, UR3A.model, brick{1}, brickDestiny{1}, qmatrixDes_{2}, UR5A.model, brick{5}, brickDestiny{5});
        brickDeposited = true;
        if brickDeposited == true

            %return arms to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.doubleMotion(restoreQMatrixUR3, UR3A.model, restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end

    end

    %approach brick
    q1_{3} = UR3A.model.getpos();
    [q2_{3}, err2_{3}, exitflag_{3}] = UR3A.model.ikcon(brick{2}.pose, q1_{3});
    qmatrix_{3} = robotMotion.interpolateJointAnglesUR3(q1_{3}, q2_{3}, steps); %Create Qmatrix based on interpolation

    %approach brick
    q1_{4} = UR5A.model.getpos();
    [q2_{4}, err2_4, exitflag_4] = UR5A.model.ikcon(brick{6}.pose, q1_{4});
    qmatrix_{4} = robotMotion.interpolateJointAnglesLinearUR5(q1_{4}, q2_{4}, steps); %Create Qmatrix based on interpolation

    %move arms
    robotMotion.doubleMotion(qmatrix_{3}, UR3A.model, qmatrix_{4}, UR5A.model)
    brickCollected = true;

    if brickCollected == true

        %drop bricks
        q3_{3} = qmatrix_{3}(end, :);
        [q4_{3}, err4_{3}, exitflag4_{3}] = UR3A.model.ikcon(brickDestiny{2}, q3_{3});
        qmatrixDes_{3} = robotMotion.interpolateJointAnglesUR3(q3_{3}, q4_{3}, steps); %Create Qmatrix based on interpolation
        q3_{4} = qmatrix_{4}(end, :);
        [q4_{4}, err4_{4}, exitflag4_{4}] = UR5A.model.ikcon(brickDestiny{6}, q3_{4});
        qmatrixDes_{4} = robotMotion.interpolateJointAnglesLinearUR5(q3_{4}, q4_{4}, steps); %Create Qmatrix based on interpolation

        %move arms
        robotMotion.doubleObjectMotion(qmatrixDes_{3}, UR3A.model, brick{2}, brickDestiny{2}, qmatrixDes_{4}, UR5A.model, brick{6}, brickDestiny{6});
        brickDeposited = true;
        if brickDeposited == true

            %return arms to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.doubleMotion(restoreQMatrixUR3, UR3A.model, restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end

    end

    %approach brick
    q1_{5} = UR3A.model.getpos();
    [q2_{5}, err2_5, exitflag_5] = UR3A.model.ikcon(brick{3}.pose, q1_{5});
    qmatrix_{5} = robotMotion.interpolateJointAnglesUR3(q1_{5}, q2_{5}, steps); %Create Qmatrix based on interpolation

    %approach brick
    q1_{6} = UR5A.model.getpos();
    [q2_{6}, err2_6, exitflag_6] = UR5A.model.ikcon(brick{7}.pose, q1_{6});
    qmatrix_{6} = robotMotion.interpolateJointAnglesLinearUR5(q1_{6}, q2_{6}, steps); %Create Qmatrix based on interpolation

    %move arms
    robotMotion.doubleMotion(qmatrix_{5}, UR3A.model, qmatrix_{6}, UR5A.model)
    brickCollected = true;

    if brickCollected == true

        %drop bricks
        q3_{5} = qmatrix_{5}(end, :);
        [q4_{5}, err4_{5}, exitflag4_{5}] = UR3A.model.ikcon(brickDestiny{3}, q3_{5});
        qmatrixDes_{5} = robotMotion.interpolateJointAnglesUR3(q3_{5}, q4_{5}, steps); %Create Qmatrix based on interpolation
        q3_{6} = qmatrix_{6}(end, :);
        [q4_{6}, err4_{6}, exitflag4_{6}] = UR5A.model.ikcon(brickDestiny{7}, q3_{6});
        qmatrixDes_{6} = robotMotion.interpolateJointAnglesLinearUR5(q3_{6}, q4_{6}, steps); %Create Qmatrix based on interpolation

        %move arms
        robotMotion.doubleObjectMotion(qmatrixDes_{5}, UR3A.model, brick{3}, brickDestiny{3}, qmatrixDes_{6}, UR5A.model, brick{7}, brickDestiny{7});
        brickDeposited = true;
        if brickDeposited == true

            %return arms to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.doubleMotion(restoreQMatrixUR3, UR3A.model, restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{7} = UR3A.model.getpos();
    [q2_{7}, err2_7, exitflag_7] = UR3A.model.ikcon(brick{4}.pose, q1_{7});
    qmatrix_{7} = robotMotion.interpolateJointAnglesUR3(q1_{7}, q2_{7}, steps); %Create Qmatrix based on interpolation

    %approach brick
    q1_{8} = UR5A.model.getpos();
    [q2_{8}, err2_8, exitflag_8] = UR5A.model.ikcon(brick{8}.pose, q1_{8});
    qmatrix_{8} = robotMotion.interpolateJointAnglesLinearUR5(q1_{8}, q2_{8}, steps); %Create Qmatrix based on interpolation

    %move arms
    robotMotion.doubleMotion(qmatrix_{7}, UR3A.model, qmatrix_{8}, UR5A.model)
    brickCollected = true;

    if brickCollected == true

        %drop bricks
        q3_{7} = qmatrix_{7}(end, :);
        [q4_{7}, err4_{7}, exitflag4_{7}] = UR3A.model.ikcon(brickDestiny{4}, q3_{7});
        qmatrixDes_{7} = robotMotion.interpolateJointAnglesUR3(q3_{7}, q4_{7}, steps); %Create Qmatrix based on interpolation
        q3_{8} = qmatrix_{8}(end, :);
        [q4_{8}, err4_{8}, exitflag4_{8}] = UR5A.model.ikcon(brickDestiny{8}, q3_{8});
        qmatrixDes_{8} = robotMotion.interpolateJointAnglesLinearUR5(q3_{8}, q4_{8}, steps); %Create Qmatrix based on interpolation

        %move arms
        robotMotion.doubleObjectMotion(qmatrixDes_{7}, UR3A.model, brick{4}, brickDestiny{4}, qmatrixDes_{8}, UR5A.model, brick{8}, brickDestiny{8});
        brickDeposited = true;
        if brickDeposited == true

            %return arms to original pose
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos, qUR3still, steps); %Create Qmatrix based on interpolation
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.doubleMotion(restoreQMatrixUR3, UR3A.model, restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;
        end
    end

    %approach brick
    q1_{9} = UR5A.model.getpos();
    [q2_{9}, err2_{9}, exitflag_{9}] = UR5A.model.ikcon(brick{9}.pose, q1_{9});
    qmatrix_{9} = robotMotion.interpolateJointAnglesLinearUR5(q1_{9}, q2_{9}, steps); %Create Qmatrix based on interpolation

    %move arm
    robotMotion.motion(qmatrix_{9}, UR5A.model);
    brickCollected = true;

    if brickCollected == true

        %drop brick
        q3_{9} = qmatrix_{9}(end, :);
        [q4_{9}, err4_{9}, exitflag4_{9}] = UR5A.model.ikcon(brickDestiny{9}, q3_{9});
        qmatrixDes_{9} = robotMotion.interpolateJointAnglesLinearUR5(q3_{9}, q4_{9}, steps); %Create Qmatrix based on interpolation
        robotMotion.objectMotion(qmatrixDes_{8}, UR5A.model, brick{9}, brickDestiny{9});
        brickDeposited = true;

        if brickDeposited == true

            %return arms to original pose
            restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos, qUR5still, steps); %Create Qmatrix based on interpolation
            robotMotion.motion(restoreQMatrixUR5, UR5A.model);
            brickCollected = false;
            brickDeposited = false;

        end
    end

    disp('Wall Built')

    %Mode 3 will be for automated way. NOte BRicks animation does not work.
    %Thus run only mode 2
elseif mode == 3

    %Create arrays for q matrixes for UR3 and UR5
    qUR3 = [];
    qUR5 = [];

    %iterate between number or bricks of UR3 and store them in a data
    %structure
    for i = 1:NumberUR3Bricks
        q1_{i} = UR3A.model.getpos();
        [q2_{i}, err1_{7}, exitflag_{7}] = UR3A.model.ikcon(UR3Bricks(i).pose, q1_{i});

        %Stack Qmatrix based on interpolation
        qmatrix_{i} = robotMotion.interpolateJointAnglesUR3(q1_{i}, q2_{i}, steps); %#ok<*SAGROW>
        qUR3 = [qUR3; qmatrix_{i}]; %#ok<*AGROW>
        q3_{i} = qmatrix_{i}(end, :);
        [q4_{i}, err2_{i}, exitflag2_{i}] = UR3A.model.ikcon(UR3Bricks(i).destiny, q3_{i});

        %Stack Qmatrix based on interpolation
        qmatrixDes_{i} = robotMotion.interpolateJointAnglesUR3(q3_{i}, q4_{i}, steps);
        qUR3 = [qUR3; qmatrixDes_{i}];

        %Stack Qmatrix based on interpolation
        restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(qmatrixDes_{i}(end, :), qUR3still, steps);
        qUR3 = [qUR3; restoreQMatrixUR3];
    end

    %iterate between number or bricks of UR5 and store them in a data
    %structure
    for i = 1:NumberUR5Bricks
        q5_{i} = UR5A.model.getpos();
        [q6_{i}, err3_{7}, exitflag3_{7}] = UR5A.model.ikcon(UR5Bricks(i).pose, q5_{i});

        %Stack Qmatrix based on interpolation
        qmatrixA_{i} = robotMotion.interpolateJointAnglesLinearUR5(q5_{i}, q6_{i}, steps);
        qUR5 = [qUR5; qmatrixA_{i}];
        q7_{i} = qmatrixA_{i}(end, :);
        [q8_{i}, err4_{i}, exitflag4_{i}] = UR5A.model.ikcon(UR5Bricks(i).destiny, q7_{i});

        %Stack Qmatrix based on interpolation
        qmatrixDesA_{i} = robotMotion.interpolateJointAnglesLinearUR5(q7_{i}, q8_{i}, steps);
        qUR5 = [qUR5; qmatrixDesA_{i}];

        %Stack Qmatrix based on interpolation
        restoreQMatrixUR5 = robotMotion.interpolateJointAnglesLinearUR5(qmatrixDesA_{i}(end, :), qUR5still, steps);
        qUR5 = [qUR5; restoreQMatrixUR5];
    end

    %get the sizes of the arrays
    sx = size(qUR3);
    sy = size(qUR5);

    %Adjust and concatonate both matrixes for double motion
    a = max(sx(1), sy(1));
    completeMatrixPositions = [[qUR3; zeros(abs([a, 0] - sx))], [qUR5; zeros(abs([a, 0] - sy))]];

    %arms motion
    robotMotion.doubleMotionSingleMatrix(completeMatrixPositions, UR3A.model, UR5A.model);

end

%% ROS BAG

clc
%Load bag
bag = rosbag('2018-03-20-18-34-46 .bag');
%select topics
topics = select(bag, 'Topic', '/joint_states');
%select messages
messages = readMessages(topics);
%Resolution was added in order to increase animation speed
resolution = 10;

dataMode = 1; % mode will be based for normal given data set or we take the transpose of that.
%Mode 1  normal data. Mode 2 will be for transpose of ROS bag

if dataMode == 1
    for i = 1:(size(messages, 1) / resolution) %filter messages in order to maximise speed of animation.
        if i == 1
            %Create Q matrix from messages
            qMatrixBag(i, :) = (messages{i}.Position);
        else
            %Keep adding elements to q matrix based on the resolution
            qMatrixBag(i, :) = (messages{i * resolution}.Position);

        end
    end

elseif dataMode == 2
    for i = 1:(size(messages, 1) / resolution) %filter messages in order to maximise speed of animation.
        if i == 1
            %Create Q matrix from messages
            qMatrixBag(i, :) = (messages{i}.Position)';
        else
            qMatrixBag(i, :) = (messages{i * resolution}.Position)'; %Keep adding elements to q matrix based on the resolution

        end
    end
end

for i = 1:size(qMatrixBag, 1)
    UR3A.model.animate(qMatrixBag(i, :)); % animate robot
    text = ['Step number: ', num2str(i)]; % display State
    disp(text); %display steps from q matrix
    drawnow() %update the figure
    pause(0.001);
end
