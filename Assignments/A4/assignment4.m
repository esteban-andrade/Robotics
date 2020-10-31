close all
clear all
clc
set(0,'DefaultFigureWindowStyle','docked');
view(3);
axis image
mdl_puma560;
hold on
%% tool Offset
toolOffsetT = transl(0,0,0.2)*troty(deg2rad(-45))
p560.tool = toolOffsetT;
%disp(p560.tool)
initialJointAngle  = [0, pi/4, -pi, 0, -pi/4, 0];
p560.plot(initialJointAngle);
axis image
%%
drumDimensions = [0.298,0.599,0.598]
endEffectorPoseRelativeToBase = p560.fkine(p560.getpos)
drumToEndEffectorOffset = 1+ endEffectorPoseRelativeToBase(3,4)-drumDimensions(1,3)
drumWindowToBaseOffset = drumToEndEffectorOffset+endEffectorPoseRelativeToBase(1,4)

%% Student ID 12824583;
clf
set(0,'DefaultFigureWindowStyle','docked');
view(3);
robotBaseT = transl([1.282, 4.583, 1]);   % Student ID 12824583
p560.base = robotBaseT;
p560.plot(initialJointAngle)
%%
xlim([0.5, 3]);
ylim([3.5, 5.5]);
zlim([0, 2]);
view(30,30)

%%
drumWindowOffset = transl(-0.298,-0.599,0);
robotToDrum = transl(drumWindowToBaseOffset,endEffectorPoseRelativeToBase(2,4),-1);
robotToDrumBase =drumWindowOffset*robotToDrum;
drumTransform = robotBaseT*robotToDrumBase;
%%

drumPosition =drumTransform(1:3,4)'
drumOrientation = tr2rpy(drumTransform);
drum = RecontructObject('Drum.ply',drumTransform);

%% Windows corner. 
windowDimensions = [0.189, 0.144]

cornerxA = robotBaseT(1,4)+drumWindowToBaseOffset-windowDimensions(2)/2
cornerxB = robotBaseT(1,4)+drumWindowToBaseOffset+windowDimensions(2)/2
corneryA = robotBaseT(2,4)+endEffectorPoseRelativeToBase(2,4)-windowDimensions(1)/2
corneryB = robotBaseT(2,4)+endEffectorPoseRelativeToBase(2,4)-windowDimensions(1)/2
cornerz = drumDimensions(3)
%%
cornerPoseA = transl(cornerxA ,corneryA,cornerz+drumToEndEffectorOffset)
toolPoseAtA = cornerPoseA* trotx(pi)* trotz(-pi/4)
jointsAtCornerA = p560.ikcon(toolPoseAtA,initialJointAngle)
p560.plot(jointsAtCornerA);
%%
cornerPoseB = transl(cornerxB ,corneryB,cornerz+drumToEndEffectorOffset)
toolPoseAtB = cornerPoseB*  trotx(pi)* trotz(-pi/4)
jointsAtCornerB = p560.ikcon(toolPoseAtB,initialJointAngle)
p560.plot(jointsAtCornerB);
%%
payloadMass = 2.09;
payloadWeight = payloadMass * 9.81;
blastingForce = 209;
p560.payload(payloadMass,[0;0;0]);  
w = [0, 0, -blastingForce, 0, 0, 0]';
tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560
velocity_max = [8, 10, 10, 5, 5, 5]';
acceleration_max = [10, 12, 12, 8, 8, 8]';
joint_angle_limits = p560.qlim
%%%%%%%%%% Variables to change %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeMin = 0.824;                                                                  % Total time to execute the motion
timeMax = 0.825;

time = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 1/100;                                                                 % Set control frequency at 100Hz
steps = round(time/dt);                                                            % No. of steps along trajectory

q = jtraj(jointsAtCornerA,jointsAtCornerB,steps);   % Quintic polynomial profile

qd = zeros(steps,6);                           % Array of joint velocities
qdd = nan(steps,6);                            % Array of joint accelerations
tau = nan(steps,6);                            % Array of joint torques
p560.payload(payloadMass,[0;0;0]);             % Set payload mass in Puma 560 model

for i = 1:steps-1
    qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));   % Calculate joint acceleration to get to next set of joint angles
    M = p560.inertia(q(i,:));                                 % Calculate inertia matrix at this pose
    C = p560.coriolis(q(i,:),qd(i,:));                        % Calculate coriolis matrix at this pose
    J = p560.jacob0(q(i,:));
    g = p560.gravload(q(i,:)) + (J'*w)';                                     % Calculate gravity vector at this pose
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed

    for j = 1:6
        % Check within max tau
        if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
            tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
            disp("Max torque reached on joint " + j + " when total time = " + time + " at t = " + i * dt)
        end

        % Check within max velocities
        if abs(qd(i,j)) > velocity_max(j)                                       % Check if v exceeds limits
            qd(i,j) = sign(qd(i,j))*velocity_max(j);                           % Cap joint v if above limits
            disp("Max velocity reached on joint " + j + " when total time = " + time + " at t = " + i * dt)
        end

        % Check within max accelerations
        if abs(qdd(i,j)) > acceleration_max(j)                                       % Check if a exceeds limits
            qdd(i,j) = sign(qdd(i,j))*acceleration_max(j);                           % Cap joint a if above limits
            disp("Max acceleration reached on joint " + j + " when total time = " + time + " at t = " + i * dt)
        end
    end
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';    % Re-calculate acceleration based on actual torque
    q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);        % Update joint angles based on actual acceleration
    qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                     % Update the velocity for the next pose

end

t = 0:dt:(steps-1)*dt;                                                      % Generate time vector
disp("DONE")
for i = 1:steps
    p560.animate(q(i,:));
    pause(0.001);
end


%% Visulalisation and plotting of results

% Plot joint angles
figure(2)
for j = 1:6
    subplot(3,2,j)
    plot(t,q(:,j)','k','LineWidth',1);
    refline(0,p560.qlim(j,1));
    refline(0,p560.qlim(j,2));
    ylabel('Angle (rad)');
    box off
end

% Plot joint velocities
figure(3)
for j = 1:6
    subplot(3,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
    refline(0,0);
    ylabel('Velocity (RPM)');
    box off
end

% Plot joint acceleration
figure(4)
for j = 1:6
    subplot(3,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('rad/s/s');
    refline(0,0)
    box off
end

% Plot joint torques
figure(5)
for j = 1:6
    subplot(3,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,tau_max(j));
    refline(0,-tau_max(j));
    ylabel('Nm');
    box off
end
