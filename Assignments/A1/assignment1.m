%% UR3 model

clear all
clc
clf

set(0,'DefaultFigureWindowStyle','docked');
view(3);
UR3A = UR3;
hold on
UR5A = UR3;%change to UR5 later
axis image
baseATransform = transl(-0.5, 0,0);
baseBTransform = transl(0.5, 0,0);
centreA = baseATransform(:,4);
centreB= baseBTransform(:,4);
centre = (centreA+centreB)/2;
UR3A.model.base = baseATransform;
UR5A.model.base = baseBTransform;
%UR3 Joint angles zero state
q = zeros(1,6);
animate(UR3A.model, q);
hold on
animate(UR5A.model, q);
% UR3A.model.teach();
% UR3B.model.teach();

%% Loading Enviroment

disp('Loading Models and Enviroment');



middlePointInBetweenRobot= middlePointInBetween(baseATransform,baseBTransform);
tablePosition =[middlePointInBetweenRobot(1),middlePointInBetweenRobot(2),middlePointInBetweenRobot(3)-0.356];
tableOrientation =[0,0,0];
Table = RecontructObject('table.ply',tableOrientation,tablePosition);

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

%Adding Person
[f,v,data] = plyread('full_body.ply','tri');



% Then plot the trisurf
person = trisurf(f,v(:,1)+1.5,v(:,2)+1.5, v(:,3) ...
    ,'EdgeColor','interp','EdgeLighting','flat');


disp('Enviroment Imported');
