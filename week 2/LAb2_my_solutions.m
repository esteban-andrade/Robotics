function Lab2MySol()
%close all
clear all
%ALL POINTS ARE TRANSFORMS
%%Q1.1 Start at the origin and moves up to 10m off the ground (positive Z)
endpoint = transl(0,0,10);
origin = transl(0,0,0); % we can use eye(4)
view(3); %for viewing in 3D
tranimate(origin, endpoint,'fps',30);
message = sprintf([num2str(round(endpoint(1,:),2,'significant')),'\n' ...
    ,num2str(round(endpoint(2,:),2,'significant')),'\n' ...
    ,num2str(round(endpoint(3,:),2,'significant')),'\n'...
    ,num2str(round(endpoint(4,:),2,'significant'))]);
text_h = text(5, 5, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
drawnow();

% Q1.2 Rotate (roll) around the X axis by -10 degrees so the Y axis is pointing more towards the ground than before
start_point = endpoint;
endpoint = start_point*trotx(deg2rad(-10));
tranimate(start_point,endpoint,'fps',30);

% Q1.3 Move in the direction of global Y to [0,2,10]
start_point = endpoint;
endpoint = transl([0,2,10])*trotx(deg2rad(-10));
tranimate(start_point,endpoint,'fps',30);

% 1.4 Roll back to level (so the orientation is now eye(3))
start_point=endpoint;
endpoint = transl([0,2,10]);
tranimate(start_point,endpoint,'fps',30);
% 1.5 Rotate (pitch) around the Y axis by 20 degrees so the X axis is pointing more towards the ground than before
start_point=endpoint;
endpoint = transl([0,2,10])*troty(deg2rad(20));
tranimate(start_point,endpoint,'fps',30);

% 1.6 Move in the direction of global X to [2,2,10]
start_point=endpoint;
endpoint = transl([2,2,10])*troty(deg2rad(20));
tranimate(start_point,endpoint,'fps',30);

% 1.7 Roll back to level (so the orientation is now eye(3))
start_point=endpoint;
endpoint = transl([2,2,10]);
tranimate(start_point,endpoint,'fps',30);

% 1.8 Go to the ground so that the new position is [2,2,0]
start_point=endpoint;
endpoint = transl([2,2,0]);
tranimate(start_point,endpoint,'fps',30);



%% Section 2

%2.3 Create and plot an instance of RobotCows using: cowHerd = RobotCows();
cowHerd = RobotCows();

%2.4 You can check the default cow count with: cowHerd.cowCount
cowHerd.cowCount

%2.5 And plot the random walk movement of them with: cowHerd.PlotSingleRandomStep();
Number =50;
for i=1:Number
    cowHerd.PlotSingleRandomStep();
end

%2.6 Increase the number of cows, by first close all; clear all; then recreate a herd by passing in a new cowCount as follows: cowHerd = RobotCows(10);
close all;
clear all;

%2.7
Number =50;
try
    delete (cowHerd);
end;
cowHerd = RobotCows(10);
delay =0.01; %(in seconds)
cowHerd.TestPlotManyStep(Number,delay);

%Query the location of the 2nd cow with: cowHerd.cow{2}.base Will give  a transform
cowHerd.cow{2}.base;

%% Section 3
% 3.1 Create a cow herd with more than 2 cows.
clc
clear all
close all
cowHerd = RobotCows(5);

%3.2 Plot the transform of the UAV starting at the origin (same as exercise 1)
UAVTR{1} = transl([0,0,0]); % Same as eye 4
trplot(UAVTR{1});

% 3.3 Determine the transform between the UAV and each of the cows

for cowIndex = 1:cowHerd.cowCount
    disp(['At Trajectory ',num2str(1),' The UAV TR to cow ',num2str(cowIndex),' is']);
    display(num2str(inv(UAVTR{1})*cowHerd.cow{cowIndex}.base));
end
cowHerd.PlotSingleRandomStep();

% 3.4 Each time the UAV moves also move the cows randomly with: cowHerd.PlotSingleRandomStep();
% 3.5 Fly through the flight path from exercise 1 and at each of the goal location determine the transform between the UAV and all the cows
UAVTR{2}=transl(0,0,10);
UAVTR{3}=transl([0,0,10])*trotx(deg2rad(-10));
UAVTR{4}= transl([0,2,10])*trotx(deg2rad(-10));
UAVTR{5}=transl([0,2,10]);
UAVTR{6}=transl([0,2,10])*troty(deg2rad(20));
UAVTR{7}= transl([2,2,10])*troty(deg2rad(20));
UAVTR{8}= transl([2,2,10]);
UAVTR{9}= transl([2,2,0]);

for trajectory = 1:size(UAVTR,2)-1
    tranimate(UAVTR{trajectory},UAVTR{trajectory+1},'fps',30);
    cowHerd.PlotSingleRandomStep();
    for cowIndex = 1:cowHerd.cowCount
        disp(['At Trajectory ',num2str(trajectory+1),' The UAV TR to cow ',num2str(cowIndex),' is']);
        display(num2str(inv(UAVTR{trajectory+1})*cowHerd.cow{cowIndex}.base));
    end
end
%% 3.6 Create a cow herd with 1 cow and move your drone so that at each step the cow moves stay 5 meters above it but directly overhead
clf
clear all
clc
cowHerd  = RobotCows(1);
UAVTR_START = eye(4);
UAVTR_GOAL = transl([0,0,5]);
tranimate(UAVTR_START,UAVTR_GOAL,'fps',30);
% Lets make iterations
for i =1:100
    cowHerd.PlotSingleRandomStep();
    UAVTR_START=UAVTR_GOAL;
    UAVTR_GOAL=cowHerd.cow{1}.base*transl([0,0,5]);
    tranimate(UAVTR_START,UAVTR_GOAL,'fps',30);
end
%% Derive the DH parameters for the simple 3 link manipulator provided. Use these to generate a model of the manipulator using the Robot Toolbox in MATLAB.

clc;
%clf;
clear all;
view(3)


% 4.1 Work out the DH Parameters.
L1 =  Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);

% 4.2 In MATLAB, run the Robot Toolbox, and generate the robot model. Check to see if it matches the examples
% provided.
% L1 = Link('d',___,'a',___,'alpha',___,'offset',___,'qlim', [__,__]);
% robot = SerialLink([L1 ... Ln],'name','myRobot');
% q = zeros(1,n); % This creates a vector of n joint angles at 0.
% workspace = [-x +x –y +y –z +z];
% scale = 1;
% robot.plot(q,'workspace’,workspace,’scale’,scale);
robot = SerialLink([L1 L2 L3],'name','my Example');   % Generate the model
q=zeros(1,3);  % Create a vector of initial joint angles
workspace = [-4 4 -4 4 -4 4]; %workspace of the robot
scale = 0.25; %scale of the joints
robot.plot(q,'workspace',workspace,'scale', scale);

% 4.3 You can manually play around with the robot: robot.teach();
robot.teach();

%4.4 Get the current joint angles based on the position in the model. q = robot.getpos()
q = robot.getpos()

% 4.5 Get the joint limits
robot.qlim
