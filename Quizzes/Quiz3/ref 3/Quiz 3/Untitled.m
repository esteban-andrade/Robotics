%%
%5DOF

L1 = Link('d',0,'a',1,'alpha',0,'offset',0);
L2 = Link('d',0,'a',1,'alpha',0,'offset',0);
L3 = Link('d',0,'a',1,'alpha',0,'offset',0);
L4 = Link('d',0,'a',1,'alpha',0,'offset',0);
L5 = Link('d',0,'a',1,'alpha',0,'offset',0);

q = [60, -45, 30,-60,0];
U = SerialLink([L1 L2 L3 L4 L5]);   
M = U.fkine(deg2rad(q));
x = M(1,4);
y = M(2,4);
z = M(3,4);
answer = [x y z]
%%
%puma560 with ball

mdl_puma560

q = [0, 30, -80, 0, 45, 0] %workspace vector
endTool_world = p560.fkine(deg2rad(q)); %base location of the puma560
ball = transl(0.4,-0.2,0.7) * trotx(-pi/2) %global transform of the ball



end_ball = inv(endTool_world)*ball %end effector tool with respect to the ball
%%
mdl_puma560
p560.base = transl(0.6, -0.1, -0.2);
p = p560.base
% E = [0.6 -0.1 -0.2]
% q = p.ikine(E,a,[1,1,1,0,0,1]);
q = p.ikine(p);

%%
mdl_puma560
p560.base = transl(0, 0, 0);
T2 = transl(0.6, -0.1, -0.2);  
q2 = p560.ikine(T2,q,[0,0,0,0,0,0])

