clf 
clear all 
close all

mdl_puma560
p560.base = transl(0,0,0.8);
q =  [90, 45, 45, 0, 45, 0] 

p560.plot(q)

p560.teach
angles = p560.getpos();


T = p560.fkine(angles)
t = T(1:3, 4)
A = transl ( 0.5,0,0.6)*trotx(pi/2)
a = A(1:3, 4)

ans = T/A

ans1 = inv(T)/A

 
R = T(1:3,1:3); 
t = T(1:3,4); 
invT = [R' -R'*t; zeros(1,3) 1]; 
ans2 = invT * A


% R2 = T(1:3,1:3);
% t = T(1:3,1:3);
% invT = [R2' - R2'*t; zeros(1,3) 1];
% ans2 = invT*A
%J = p560.jacob0(angles)
%J = J(1:3,1:3)