%% %% determine the distance between the "base"...
clc
clear all
clf

mdl_baxter()

left
right 

% left.teach();
% 
% hold on 
% 
% right.teach();

display ('Done');

workspace = [-3 3 -3 3 -3 3];
%% Change XYZ if you need to transform the base
X = 0
Y = 0
Z = 0

left.base = transl(0.064614+X, 0.25858+Y, 0.119+Z)*rpy2tr(0, 0, pi/4);
right.base = transl(0.063534+X, -0.25966+Y, 0.119+Z)*rpy2tr(0, 0, -pi/4);

lBasePos = left.base;
rBasePos = right.base ;

%only want the xyz part
xyzLeftBase = lBasePos(1:3,4)
hold on
xyzRightBase = rBasePos(1:3,4)

%distance between 2 pts is the square rt of delta x^2 dY^2 and dZ^2
%baseDistance = sqrt(((xyzLeft(1,1)-xyzRight(1,1))^2) + ((xyzLeft(2,1)-xyzRight(2,1))^2) + ((xyzLeft(3,1)-xyzRight(3,1))^2))%% Change the angles

%setAngles 7 angles
% qSetLeft = [5*pi/3,0,0,0,8*pi/2,0,0]
% qSetRight = [7*pi/4,0,0,0,0,0,-3*pi/20]
% qSetLeft =  [0,-pi/2,0,0,6*pi/8,0,0];
% 
qSetLeft = [0,0,0,0,0,0,0];
% qSetLeft =  [5*pi/3,0,0,0,8*pi/2,0,0];
qSetRight =  [-9*pi/10,0,0,4*pi/9,0,0,0];


 left.plot(qSetLeft, 'workspace', workspace, 'scale', 0.3);
 right.plot(qSetRight, 'workspace',workspace, 'scale', 0.3);

 %left.teach();
 %right.teach();
%get angles
qLeft  = left.getpos();
qRight = right.getpos();

%get xyz location
posLeft = left.fkine(qLeft);
posRight = right.fkine(qRight);

%only want the xyz part
xyzLeft = posLeft(1:3,4)
xyzRight = posRight(1:3,4)



%distance between 2 pts is the square rt of delta x^2 dY^2 and dZ^2
handToHandDistance = sqrt(((xyzLeft(1,1)-xyzRight(1,1))^2) + ((xyzLeft(2,1)-xyzRight(2,1))^2) + ((xyzLeft(3,1)-xyzRight(3,1))^2))
lHand2LBaseDistance = sqrt(((xyzLeft(1,1)-xyzLeftBase(1,1))^2) + ((xyzLeft(2,1)-xyzLeftBase(2,1))^2) + ((xyzLeft(3,1)-xyzLeftBase(3,1))^2))
lhand2RBaseDistance = sqrt(((xyzLeft(1,1)-xyzRightBase(1,1))^2) + ((xyzLeft(2,1)-xyzRightBase(2,1))^2) + ((xyzLeft(3,1)-xyzRightBase(3,1))^2))
RHand2LBaseDistance = sqrt(((xyzRight(1,1)-xyzLeftBase(1,1))^2) + ((xyzRight(2,1)-xyzLeftBase(2,1))^2) + ((xyzRight(3,1)-xyzLeftBase(3,1))^2))
Rhand2RBaseDistance = sqrt(((xyzRight(1,1)-xyzRightBase(1,1))^2) + ((xyzRight(2,1)-xyzRightBase(2,1))^2) + ((xyzRight(3,1)-xyzRightBase(3,1))^2))

%  %% ikine trajectory
% % %q = [0 0 0 0 0 0 0]
% q = zeros(1,7)
% T1 = transl(0,-1.5,0);
% q1 = left.ikine(T1,q,[1,1,1,0,0,0]);%%xyzrpy
% T2 = transl(-1.5,0,0);
% q2 = left.ikine(T2,q1,[1,1,1,0,0,0]); %% q1 because thats where i am coming from.
% % 
% steps = 50
% s = lspb(0,1,steps);
% qMatrix1 = nan(steps,7)
% for i = 1:steps
%     qMatrix1(i,:) = (1-s(i))*q1 + s(i)*q2  
%     %q(s) = (1-s)qa + s*qb
%     % t = robot.fkine(qMatrix1(i,:));
%     % p(i,:) = t(1:3,4)
%     % : represents all rows and all columns.
%     % creates a matrix of all joint states for each step from 1 - 50.
%     % when s = 0 q(s) = qa
%     % when s = 1 q(s) =qb
%     % used for the joint state positions between two points q1 and q2.
% end
% hold on
% left.plot(qMatrix1)
% %right.plot(qMatrix1,'trial','r')
% hold off
% 