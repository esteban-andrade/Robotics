%% mdl_hyper2d should be 10 joints visible - run teach and observe/count
clc
clear all 
clf
view(3)
mdl_hyper2d;
h2d.plot([pi, pi/2, -pi/2, -pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/3, -pi/3]);
h2d.teach

% T(from J to J-1) = Trz(thetaj)*Tz(dj)*Tx(aj)*Trx(alphaj) 

%  Rotation in z*Trans in z* trans in x* and rotation in x 

% In this convention, coordinate frames are attached to the joints between two links such that one transformation
%%
%%%%%%% PUMA WITH BALL ON TABLE %%%%%%%
clf
mdl_puma560
p560
% p560.teach();

changeBase = true;
% New base position as [x y z]
basePosition = [0 0 0.8];

% New base orientation as [roll pitch yaw] in degrees
baseOrientation = [0 0 0];

% Ball global transform
ball = transl(0.7,0,0.4)*trotx(pi/2);

if changeBase
    p560.base = transl(basePosition) * rpy2tr(baseOrientation, 'deg');
end

q = [45,45,45,0,45,0];
endTool_world = p560.fkine(deg2rad(q))
end_ball = inv(endTool_world)*ball

% p560.plot(q)
% p560.teach
%%
%%%%%%% PUMA WITH FKINE True/False %%%%%%%
clf
mdl_puma560
p560
fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0])))
fkine(p560,[pi/4,0,0,0,0,0])
% Compare in command window and decide


%%%%%%% PUMA Middle of range %%%%%%%
clf
mdl_puma560
p560
q = p560.qlim
p560.jacob0(q)
p560.teach

%%%%%%% PUMA Bolted to the floor Distance %%%%%%%
clf
mdl_puma560
p560
q = [0,pi/10,0,0,0,0];
p560.plot(q);
p560.teach
%%
%%%%%%% PUMA Bolted to table distance to floor %%%%%%%
basePosition = [0 0 0.6]; % ie distance off the floor
clf
mdl_puma560
p560
p560.base = transl(basePosition) * rpy2tr(baseOrientation, 'deg');
q = [0,0,0,pi/10,0,0];
p560.plot(q);
p560.teach
%%
%%%%%%% Baxter left and right end effector distance %%%%%%%
clf
mdl_baxter
qLeft = [3*pi/10,0,0,0,0,0,2*pi/10]
qRight = [4*pi/10,0,0,-2*pi/5,0,0,0]
tleft=left.fkine(qLeft)
tright=right.fkine(qRight)
dist=norm(tleft(1:3,4)-tright(1:3,4))
%%
%%%%%%% Baxter Left effector to Right Base dist %%%%%%%
clf
mdl_baxter
qLeft = [3*pi/10,0,0,0,0,0,2*pi/10]
tleft=left.fkine(qLeft)
dist=norm(tleft(1:3,4)-right.base(1:3,4))
%%
%%%%%%% Baxter Right effector to Left Base dist %%%%%%%
clf
mdl_baxter
qRight = [3*pi/10,0,0,0,0,0,2*pi/10]
tRight=right.fkine(qRight)
dist=norm(tRight(1:3,4)-left.base(1:3,4))
%%
%%%%%%% Baxter Unmasked (LEFT) FORWARD %%%%%%%
%%% ALWAYS IDENTICAL %%%

mdl_baxter

LEFT = 1;
RIGHT = 0;

% Set this as either LEFT or RIGHT
robotToUse = RIGHT;
if robotToUse == LEFT
    robot = left;
end
if robotToUse == RIGHT
    robot = right;
end
robot
robot.teach()
% Goal position as [x y z]
goalPosition = [0.1 0.2 0.3];

% Goal orientation as [roll pitch yaw] in degrees
goalOrientation = [0 0 0];

% Mask vector
mask = [1 1 1 0 0 0];
unmask = [1 1 1 1 1 1];

goalPose = transl(goalPosition) * rpy2tr(goalOrientation, 'deg');
goalQ = robot.fkine(goalPose, robot.getpos(), mask)

robot.animate(goalQ)
