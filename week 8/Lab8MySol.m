%% Q1 Image Based Visual Servoing

clear all
close all
clc

% Make a UR10
r = UR10();             

%Initial pose
q0 = [pi/2; -pi/3; -pi/3; -pi/6; 0; 0];

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');
fps = 25;
lambda = 0.6; 


object_height = 0.5;
object_position = [1.25,0,1]; %x, y, z
x = object_position(1);
y = object_position(2);
z = object_position(3);
h = object_height/2
p = [h+x,h+y,h+z;
    h+x,h+y,-h+z;
    h+x,-h+y,h+z;
    h+x,-h+y,-h+z;
    -h+x,h+y,h+z;
    -h+x,h+y,-h+z;
    -h+x,-h+y,h+z;
    -h+x,-h+y,-h+z]

P1 = [p(1,1:3)',p(2,1:3)',p(3,1:3)',p(4,1:3)'];
P1 = [P1(1:3,3),P1(1:3,1),P1(1:3,2),P1(1:3,4)];%adjusts rows to link correctly to pStar points
P2 = [p(1,1:3)',p(2,1:3)',p(6,1:3)',p(5,1:3)'];
P2 = [P2(1:3,3),P2(1:3,1),P2(1:3,2),P2(1:3,4)];%adjusts rows to link correctly to pStar points
P3 = [p(5,1:3)',p(6,1:3)',p(7,1:3)',p(8,1:3)'];
P3 = [P3(1:3,3),P3(1:3,1),P3(1:3,2),P3(1:3,4)];%adjusts rows to link correctly to pStar points
P4 = [p(3,1:3)',p(7,1:3)',p(8,1:3)',p(4,1:3)'];
P4 = [P4(1:3,3),P4(1:3,1),P4(1:3,2),P4(1:3,4)];

P = [1.8, -0.25, 1.25;1.8, 0.25, 1.25;1.8, 0.25, 0.75;1.8, -0.25, 0.75];


pStar = [662 362 362 662; 362 362 662 662];

Tc0 = r.model.fkine(q0);
r.model.animate(q0');
drawnow

cam.T = Tc0;
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15)

%depth = mean (P3(1,:));
plot_sphere(P', 0.05, 'g')
%plot_sphere(P2,0.05,'c')
lighting gouraud
light
depth = 1.8;
%%

%Project points to the image
p = cam.plot(P', 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P', 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P');    % show initial view


%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];

%%
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P');
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P');
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = r.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp
        r.model.animate(q');

        %Get camera location
        Tc = r.model.fkine(q);
        cam.T = Tc;

        drawnow
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes
 
 
 %%
 figure()            
plot_p(history,pStar,cam)
figure()
plot_camera(history)
figure()
plot_vel(history)
figure()
plot_robjointpos(history)
figure()
plot_robjointvel(history)
 
