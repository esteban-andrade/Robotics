%% Self collisions for a Hyper Redundant 2D arm
disp('Don''t collide')

%% D&H Transforms and forward kinematics in Matlab
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
robot = SerialLink([L1 L2 L3],'name','myRobot');

robot.base = transl(0, 0, 0);
robot.teach();

% Joint configuration
q = [0, 45, 80];

endEffector = robot.fkine(q, 'deg');

robot.animate(q);