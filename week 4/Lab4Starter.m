function Lab4Starter( )
close all;
%% Create robot and workspace
set(0,'DefaultFigureWindowStyle','docked');
view(3);
mdl_twolink

% specify workspace
workspace = [-1 2.5 -2 2 -1 2];

scale = 0.5;

qz = [0,0];

twolink.plot(qz,'workspace',workspace,'scale',scale);

twolink.teach;

hold on;
% As we move the joints, it only Moves in the XZ plane. FRom this view
% using trig we can get angles for joint 1 and joint 2 for a specified
% end-effector location


%% Calculate joint angles for a two link planar arm
% specify an end effector position
tr = [1.7 0 0.1];
x = tr(1,1);
z = tr(1,3);

% using cosine rule (a^2 = b^2 + c^2 - 2bc*cos(A)),
% where cos(pi-A) = -cos(A)
% and cosA = cos(A)
% and x^2+z^2 = a^2
% and a and b are the link lengths equal to 1m
cosA = ((x^2+z^2-1^2-1^2)/(2*1*1));

% calculate joint 2 for poses 1 and 2
pose1theta2 = (atan2(sqrt(1-cosA^2),cosA));
pose2theta2 = (atan2(-sqrt(1-cosA^2),cosA));

% calculate joint 1 for poses 1 and 2
pose1theta1 = (atan2(z,x)-atan2((1)*sin(pose1theta2),1+(1)*cos(pose1theta2)));
pose2theta1 = (atan2(z,x)-atan2((1)*sin(pose2theta2),1+(1)*cos(pose2theta2)));

pose1 = [pose1theta1, pose1theta2];
pose2 = [pose2theta1, pose2theta2];


%% Confirm joint angles using fkine
qFkine1 = twolink.fkine(pose1)
qFkine2 = twolink.fkine(pose2)

% There are 2 set of angles that can achive the end-effector location. The
% values for the rotation matrix will not match as it is the result of the
% robot contraining RPY values. Translation matrix will match
%  twolink.plot([pose1theta1 pose1theta2]) to get the robot to that
%  position
% twolink.plot([pose2theta1 pose2theta2]) for elbow up config

%% Draw straight line
% Plot a trajectory for the end-effector which moves from x = 1.7 to x = 0.7
% while maintaining a z height of 0.1

% preallocate matrix if you want to view the joint angles later
qMatrix = zeros(100, 2);
count = 1;

 newQ = twolink.ikine(transl(tr),qz,[1,0,1,0,0,0]);

for i = 1.7:-0.05:0.7
    %     cosA = ((i^2+z^2-1^2-1^2)/(2*1*1));
    %     qMatrix(count,2) = (atan2(-sqrt(1-cosA^2),cosA));
    %     qMatrix(count,1) = (atan2(z,x)-atan2((1)*sin(qMatrix(count,2)),1+(1)*cos(qMatrix(count,2))));
    %     twolink.animate(qMatrix(count,:));
    %     drawnow();
    %
    %     count = count+1;
    newQ = twolink.ikine(transl(i,0,z),newQ,[1,0,1,0,0,0]);
    % Specify point we wanna be at, current state, and mask  (dont care about y and remove RPY)
    twolink.animate(newQ);
    drawnow();
end
