clf
clear all
close all
 clc
 
set(0,'DefaultFigureWindowStyle','docked')
view (3)

%% Derive the Jacobian, ( ) by hand for a 3-Link Planar Robot using position variables, and the rotation 1.1about the z-axis variable, as the joint angle, changes
 % should be on the pptx Linear Planar Robot
 %% Now input the equation for ( ) into Matlab using symbolic values
syms l1 l2 l3 x y phi q1 q2 q3 Jq;
x = l1*cos(q1) + l2*cos(q1+q2) + l2*cos(q1+q2+q3);
y = l1*sin(q1) + l2*sin(q1+q2) + l2*sin(q1+q2+q3);
phi = q1 + q2 + q3;

% Compute the Jacobian
Jq = [diff(x,q1),diff(x,q2),diff(x,q3) ...
    ; diff(y,q1),diff(y,q2),diff(y,q3) ...
    ; diff(phi,q1),diff(phi,q2),diff(phi,q3)];

%J =[ - sin(q1 + q2 + q3) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2), -sin(q1 + q2 + q3)]
%[ cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2), cos(q1 + q2 + q3)]
%[ 1, 1, 1]

% 1.3 Solve for the link lengths being 1
JqForLength1 = subs(subs(subs(Jq,l1,1),l2,1),l3,1)

% 1.4 Solve for all joint angles being 0. By observation x velocity is 0
subs(subs(subs(JqForLength1,q1,0),q2,0),q3,0)

% Confirm this by using the toolbox
mdl_planar3;                                                                % Load 2-Link Planar Robot
p3.jacob0([0,0,0])
p3.teach

%% Dealing with Singularities
clf
clear all
clf
mdl_planar2;                                                               % Load 2-Link Planar Robot
M = [1 1 zeros(1,4)];                                                      % Masking Matrix
t = 5 ;                                                                  % Total time in seconds (try 5 sec)
steps = 100 ;                                                              % No. of steps (try 100)
deltaT = t/steps;                                                          % Discrete time step
deltaTheta = 4*pi/steps;                                                   % Small angle change
qMatrix = zeros(steps,2);                                                  % Assign memory for joint angles
x = zeros(2,steps);                                                        % Assign memory for trajectory
m = zeros(1,steps);                                                        % For recording measure of manipulability
errorValue = zeros(2,steps);                                               % For recording velocity error
minManipMeasure = 0.1; 
for i = 1:steps
    x(:,i) = [1.5*cos(deltaTheta*i) + 0.45*cos(deltaTheta*i)
              1.5*sin(deltaTheta*i) + 0.45*cos(deltaTheta*i)];
end

T = [eye(3) [x(:,1);0];zeros(1,3) 1];
%qMatrix(1,:) = p2.ikine(T,[0 0],M);
qMatrix(1,:) = p2.ikcon(T,[0 0])

for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                                      % Calculate velocity at discrete time step
    J = p2.jacob0(qMatrix(i,:));                                            % Get the Jacobian at the current state
    J = J(1:2,:);                                                           % Take only first 2 rows
    m(:,i)= sqrt(det(J*J'));                                                % Measure of Manipulability
    if m(:,i) < minManipMeasure
        qdot = inv(J'*J + 0.01*eye(2))*J'*xdot;
    else
        qdot = inv(J) * xdot;                                               % Solve velocitities via RMRC
    end
    error(:,i) = xdot - J*qdot;
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';                         % Update next joint state
end
figure(1)
view (3)
set(gcf,'units','normalized','outerposition',[0 0 1 1])
p2.plot(qMatrix,'trail','r-')                                               % Animate the robot
figure(2)
plot(m,'k','LineWidth',1);                                                  % Plot the Manipulability
title('Manipulability of 2-Link Planar')
ylabel('Manipulability')
xlabel('Step')
figure(3)
plot(error','Linewidth',1)
ylabel('Error (m/s)')
xlabel('Step')
legend('x-velocity','y-velocity');

%%  Depth Image
close all
clear all 
clc
load('imageData.mat');

for i = 1:100
    imshow(histeq(depthImageData(:,:,i)));
    imshow(rgbImageData(:,:,:,i));
    drawnow();
    pause(0.5);
end

%%
surf(histeq(depthImageData(:,:,1)));
%%
widthFOV = 58;
heightFOV = 45;
widthResolutionDegrees = widthFOV / size(depthImageData,2);
heightResolutionDegrees = heightFOV / size(depthImageData,1);

%% 3.5 (point spacing at 1m) Medium res
widthSpacing = tan(deg2rad(widthResolutionDegrees));
heightSpacing = tan(deg2rad(heightResolutionDegrees));
%% 3.7 Point Cloud
X_TO_Z = 1.114880018171494;
Y_TO_Z = 0.836160013628620;
rowCount = size(depthImageData,1);
colCount = size(depthImageData,2);
pointMillimeters = zeros(colCount*rowCount,3);
for i = 1:rowCount
    for j = 1:colCount
        x = (j /colCount -0.5) * depthImageData(i,j,1) * X_TO_Z;
        y = (0.5 -i / rowCount) * depthImageData(i,j,1) * Y_TO_Z;
        pointMillimeters((i-1) * colCount + j,:) = [x, y, depthImageData(i,j,1)];
    end
end
pointMeters = pointMillimeters / 1000

%% 3.8 (Bonus) Eye-In-Hand
robot = SchunkUTSv2_0;
q = [0,pi/2,0,0,0,0];
robot.plot3d(q);
camlight
hold on;
plot_h = plot3(0,0,0,'.r');
for joint2Rads = pi/2 : -0.1:0
    q = [0,joint2Rads,0,0,0,0];
    tr = robot.fkine(q);
    transformedPoints = [tr * [pointMeters,ones(size(pointMeters,1),1)]']';
    transformedPoints = transformedPoints(:,1:3);
    plot_h.XData = transformedPoints(:,1);
    plot_h.YData = transformedPoints(:,2);
    plot_h.ZData = transformedPoints(:,3);
    robot.animate(q);
    axis equal
    drawnow();
end