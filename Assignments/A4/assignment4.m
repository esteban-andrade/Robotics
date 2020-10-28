close all
clear all
clc
set(0,'DefaultFigureWindowStyle','docked');
view(3);
axis image
mdl_puma560;
hold on
%% tool Offset
toolOffsetT = transl(0,0,0.2)*troty(deg2rad(-45));
p560.tool = toolOffsetT;
disp(p560.tool)
%% Student ID 12824583;

robotBaseT = transl([1.282, 4.583, 1]);   % Student ID 12544944
p560.base = robotBaseT;
initialJointAngle  = [0, pi/4, -pi, 0, -pi/4, 0];
p560.plot(initialJointAngle);
axis image
%%

drumPosition = [0,0,0];
drumOrientation = [0,0,0];
drum = RecontructObject('Drum.ply',drumOrientation,drumPosition);

