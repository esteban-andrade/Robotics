%% modelhyper2D
clear all
clc
clf

mdl_hyper2d
%hyperq1 = 	[pi, pi/2, -pi/2, pi, -pi/2, pi/2, -pi/2, pi/2, -pi/3, -pi/3]
%hyperq1 = 	[pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/3, -pi/3]
%hyperq1 = 	[-pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/3, pi/3, -pi/3]
% hyperq1 = 	[-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, pi/3, -pi/3]

% hyperq1 = [-pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/3, -pi/3]
% hyperq1 = [pi, pi/2, -pi/2, -pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/3, -pi/3]
% hyperq1 =[pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/3, -pi/3]
% hyperq1 = [pi/2, pi/2, -pi/4, -pi/4, pi/4, pi/4, -pi/2, pi/2, -pi/3, -pi/3]

hyperq1 = [pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/2, -pi/3, -pi/3];


h2d.plot(hyperq1); 
%%repeatfor2,3,4