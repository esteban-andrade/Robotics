% Camera mounted on UR10, calculate feature error
clear all
clf
P=[2,2,2,2; -0.4,0.4,0.4,-0.4; 1.4,1.4,0.6,0.6]; 
pStar = [700 300 300 700; 300 300 700 700]; 
cam = CentralCamera('focal',0.08,'pixel',10e-5,'resolution',[1024 1024], 'centre',[512 512],'name','UR10camera');
r = UR10();
q0 =  [1.6; -1; -1.2; -0.5; 0; 0];
Tc0 = r.model.fkine(q0);
cam.T = Tc0;
uv = cam.plot(P);
e = pStar-uv

%  RectangularPrism and IsCollision
clear all
clf

mdl_planar3 
stepCount = 50;
q1 = [-pi/3,0,0];
q2 = [pi/3,0,0];
trej = jtraj(q1,q2,stepCount);
[v,f,fn] = RectangularPrism([2,-1,-1], [3,1,1]);
for i = 1:numel(trej)
    q = trej(i,:);
    result = IsCollision(p3,q,f,v,fn);
    if(result == 1)
        r = trej(i,:)
        break 
    end
end

% 3-Link Planar Robot Poses closes to singularity
clear all
clf
mdl_planar3
q = [0 -0.7854 -0.7854];
jacobian = p3.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))

% Puma560 closest to singularity
clear all
clf
mdl_puma560
q = [0 2.1677 -2.7332 0 -0.9425 0];
jacobian = p560.jacob0(q);
measureOfManip = sqrt(det(jacobian(1:2,:)*jacobian(1:2,:)'))