%% UR3 model

clear all
clc
clf

set(0,'DefaultFigureWindowStyle','docked');
view(3);
UR3A = UR3;
%hold on
UR5A = LinearUR5(false);%change to UR5 later
axis image
baseATransform = transl(-1,0,0);
baseBTransform = transl(1,0,0)*trotx(pi/2);%*troty(pi/2);
centreA = baseATransform(:,4);
centreB= baseBTransform(:,4);
centre = (centreA+centreB)/2;
UR3A.model.base = baseATransform;
UR5A.model.base = baseBTransform;
%UR3 Joint angles zero state
q = zeros(1,6);
p=zeros(1,7);
animate(UR3A.model, q);
hold on
animate(UR5A.model, p);
% UR3A.model.teach();
% UR3B.model.teach();

%% Loading Enviroment

disp('Loading Models and Environment');



middlePointInBetweenRobot= middlePointInBetween(baseATransform,baseBTransform);
tablePosition =[middlePointInBetweenRobot(1),middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)-0.356];
tableOrientation =[0,0,0];
Table = RecontructObject('table3.ply',tableOrientation,tablePosition);

floorPosition = [middlePointInBetweenRobot(1)+0.7,middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)];
floorOrientation =[0,pi/2,0];
Floor = RecontructObjectNonRGB('floor2.ply',floorOrientation,floorPosition);

brick1Position = [-0.2,-0.2,0.05];
brick1Orientation = [0,0,0];
brick1 = RecontructObject('Brick.ply',brick1Orientation,brick1Position);

brick2Position = [0,0,0.05];
brick2Orientation = [0,0,0];
brick2 = RecontructObject('Brick.ply',brick2Orientation,brick2Position);

brick3Position = [0.2,0.2,0.05];
brick3Orientation = [0,0,0];
brick3 = RecontructObject('Brick.ply',brick3Orientation,brick3Position);

brick4Position = [0,-0.4,0.05];
brick4Orientation = [0,0,0];
brick4 = RecontructObject('Brick.ply',brick4Orientation,brick4Position);

brick5Position = [0.2,0.4,0.05];
brick5Orientation = [0,0,0];
brick5 = RecontructObject('Brick.ply',brick5Orientation,brick5Position);

brick6Position = [-0.2,-0.2,0.12];
brick6Orientation = [0,0,0];
brick6 = RecontructObject('Brick.ply',brick6Orientation,brick6Position);

brick7Position = [0,0,0.12];
brick7Orientation = [0,0,0];
brick7 = RecontructObject('Brick.ply',brick7Orientation,brick7Position);

brick8Position = [0.2,0.2,0.12];
brick8Orientation = [0,0,0];
brick8 = RecontructObject('Brick.ply',brick8Orientation,brick8Position);

brick9Position = [0.2,0.4,0.12];
brick9Orientation = [0,0,0];
brick9 = RecontructObject('Brick.ply',brick9Orientation,brick9Position);


BobPosition = [middlePointInBetweenRobot(1),middlePointInBetweenRobot(2)+3,middlePointInBetweenRobot(3)];
BobOrientation = [0,0,0];
Bob = RecontructObjectNonRGB('full_body.ply',BobOrientation,BobPosition);

fence1Position= [middlePointInBetweenRobot(1)+2.5,middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)+0.3];
fence1Orientation = [0,0,pi/2];
fence1 = RecontructObject('fence.ply',fence1Orientation,fence1Position);

fence2Position= [middlePointInBetweenRobot(1)-2.5,middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)+0.3];
fence2Orientation = [0,0,pi/2];
fence2 = RecontructObject('fence.ply',fence2Orientation,fence2Position);

fence3Position= [middlePointInBetweenRobot(1)+2.5,middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)+0.3];
fence3Orientation = [0,0,0];
fence3 = RecontructObject('fence.ply',fence3Orientation,fence3Position);
% 
fence4Position= [middlePointInBetweenRobot(1)-2.5,middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)+0.3];
fence4Orientation = [0,0,0];
fence4 = RecontructObject('fence.ply',fence4Orientation,fence4Position);

eButtonPosition = [middlePointInBetweenRobot(1)+1.7,middlePointInBetweenRobot(2)+0.7,middlePointInBetweenRobot(3)+0.1];
eButtonOrientation = [0,0,0];
eButton = RecontructObject('emergency.ply',eButtonOrientation,eButtonPosition);


warning1Position = [middlePointInBetweenRobot(1),middlePointInBetweenRobot(2)+2.6,middlePointInBetweenRobot(3)+0.7];
warning1Orientation = [0,0,0];
warning1 = RecontructObject('warning.ply',warning1Orientation,warning1Position);

%warning2Position = [middlePointInBetweenRobot(1),middlePointInBetweenRobot(2)-2.6,middlePointInBetweenRobot(3)-0.7];
%warning2Orientation = [0,0,pi];
%warning2 = RecontructObject('warning.ply',warning2Orientation,warning2Position);


disp('Environment Imported');
