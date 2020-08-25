clear all;
clc;
clf;

mdl_puma560
q = pi/18*ones(1,6)
% q = [p560.qlim(:,1) + (p560.qlim(:,2) - p560.qlim(:,1) )/2]';
jacobMatrix = p560.jacob0(q)

q(1,1) = q(1,1)*pi/4
jacobMatrix = p560.jacob0(q)
