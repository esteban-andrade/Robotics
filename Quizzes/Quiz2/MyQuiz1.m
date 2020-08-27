%%PUMA 560

%PUMA 560 and ball
clc
clear all 
clf
view(3)
mdl_puma560;
p560;
set(0,'DefaultFigureWindowStyle','docked')
newBase = true;

basePosition = [0,0,0.5];
baseOrientation = [0,0,0];

if newBase
    p560.base=transl(basePosition)*rpy2tr(baseOrientation);
end

ball =transl(0.5,0,0.6) * trotx(pi/2) ;
trplot(ball);

%find position End Effector
q = [45, 0, 45, 0, 45, 0];

endEffector = p560.fkine(q,'deg');
p560.teach(q);
hold on 
axis image
ballRelativeToEndEffector = inv(endEffector)*ball

answer =ballRelativeToEndEffector(1:3,4)' 
%% Puma 560 check fw kinematics true
clc
clear all
clf 
view(3)
mdl_puma560;
p560;

%fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]))) == fkine(p560,[pi/4,0,0,0,0,0])

option1 =   fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0]))) 
option2 =  fkine(p560,[pi/4,0,0,0,0,0])
if option1~=option2
     disp('false')
else
     disp('true') 
end

%fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0]),[-pi/4,0,0,0,0,0])) == fkine(p560,[-pi/4,0,0,0,0,0])
option3 =  fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0])) 
option4 =   fkine(p560,[-pi/4,0,0,0,0,0])
if option3~=option4
      disp('false')
else
     disp('true')
end
%% PUMA 560 floor middle Range
clc
clear all
clf 
view(3)
mdl_puma560;
p560
%q =  [p560.qlim(:,1) + (p560.qlim(:,2)-p560.qlim(:,1))/2]';
q= pi/18*ones(1,6);
q(2)=q(2)+pi/4;
p560.jacob0(q)
p560.teach(q)
% Joint 4 5 6 only change RPY Not position in xyz

%% PUMA 560. Bolted on the floor

clc
clear all
clf 
view(3)
mdl_puma560;
p560
q = [0,pi/10,0,0,0,0];

endEffector = p560.fkine(q)
p560.plot(q);

distance = endEffector(3,4)
%% Puma 560 bolted to table

clc
clear all 
clf
view(3)
mdl_puma560;
p560;
set(0,'DefaultFigureWindowStyle','docked')
basePosition = [0,0,0.5];
baseOrientation = [0,0,0];

p560.base = transl(basePosition)*rpy2tr(baseOrientation);
q = [0,0,pi/10,0,0,0];
%q = deg2rad([45, 0, 45, 0, 45, 0])
endEffector = p560.fkine(q);
p560.plot(q);
distance = endEffector(3,4)

%% BAXTER determine the distance netween bade of lelft arm
clc
clear all 
clf
view(3)
mdl_baxter;
set(0,'DefaultFigureWindowStyle','docked')

% determine the distance between the base of the left arm and the enf
% effecyot of the right arm 
qRight = [0,3*pi/8,0,0,0,0,3*pi/4];
qLeft =  [3*pi/10,0,0,0,0,0,2*pi/10];
right.base; % right  arm position
left.base; % left arm position

trRight = right.fkine(qRight)
trLeft = left.fkine(qLeft);
distance = norm(trLeft(1:3,4)-right.base(1:3,4))

distance_mm = distance*1000
% We can change left to right to make it work backwards

%% Baxter DISTANCE berteen two end effectors.
clc
clear all 
clf
view(3)
mdl_baxter;
set(0,'DefaultFigureWindowStyle','docked')

% determine the distance between the base of the left arm and the enf
% effecyot of the right arm 
qRight =   [0,-pi/10,0,0,0,pi/10,0];
qLeft =   [0,pi/10,0,0,0,-pi/10,0] ;
right.base; % right  arm position
left.base; % left arm position

trRight = right.fkine(qRight)
trLeft = left.fkine(qLeft);
distance = norm(trLeft(1:3,4)-trRight(1:3,4))

distance_mm = distance*1000
%% BAXTER

%What can you say about the ikine solution attempting to get to tr = left.fkine(qr) when you use qr*1.1 as the initial guess and you do not mask off the roll, pitch, and yaw (i.e. by passing [1,1,1,1,1,1] to ikine for the mask)
clc
clear all 
clf
view(3)
mdl_baxter
set(0,'DefaultFigureWindowStyle','docked')
robot;

q = zeros (1,7);
tr = left.fkine(qr) % qr is a joint angle configuration where y and z are -90 degrees.

masked = [1,1,1,0,0,0];
unmasked = [1,1,1,1,1,1];

qm = left.ikine(tr,q,masked)
%qu =left.ikine(tr,q,unmasked)

tm = left.fkine(qm)
%tu = left.fkine(qu)

% Position is about the same. Orientation is different.

%% Hyper redundant planar mdlz-hyper2d

close all
set(0,'DefaultFigureWindowStyle','docked')
view(3)
clear
clc
mdl_hyper2d
q=[pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi/2, -pi/3, -pi/3] % enter the joint positions here or..
%h2d.teach(q)
h2d.plot(q)

%second option



%%
%Given only the D&H parameters for the jth link , but an unknowable joint angle, how could you encode the transform from j-1th frame to jth frame (i.e.  in matlab?

 		
%There is not enough information - I don't know the joint type or angle variable