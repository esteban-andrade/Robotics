%% Determine the distance from the end effector to the floor 
clear all
clc
clf
mdl_puma560;
q = [pi/10,0,0,0,0,2*pi/5];
% q = deg2rad([0,45,45,45,45,45])
p560.base = transl(0,0,0)
p560.fkine(q)