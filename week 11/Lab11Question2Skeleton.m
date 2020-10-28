%% Robotics
% Lab 11 - Question 2 skeleton code

%For virtual joystick
% Add pendant = VirtualTeachPendant; 
% joy = vrjoystick(id);
%dx = round(dx*1000)/1000


%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information


%% Set up robot
mdl_puma560;                    % Load Puma560
robot = p560;                   % Create copy called 'robot'
robot.tool = transl(0.1,0,0);   % Define tool frame on end-effector
set(0,'DefaultFigureWindowStyle','docked');
view(3);

%% Start "real-time" simulation
q = qn;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot
robot.plot(q);          % Plot robot in initial configuration
robot.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 1000;  % Set duration of the simulation (seconds)
dt = 0.15;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    %[axes, buttons] = read(joy);
     [axes, buttons, povs] = read(joy);  
     
     
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    % 2 - use J inverse to calculate joint velocity
    % 3 - apply joint velocity to step robot joint angles 
    % -------------------------------------------------------------
    Kv = 0.2; % Linear velocity Gain
    Kw = 1 % angular velocity gain
    
    vx=Kv*axes(1)
    vy=Kv*axes(2)
    vz=Kv*( buttons(5)-buttons(7))
    
    wx = Kw*axes(4);
    wy =Kw*axes(3);
    wz = Kw*( buttons(6)-buttons(8))
    
    dx = [vx;vy;vz;wx;wy;wz]
    dx((dx.^2)<0.01)=0;
    
    J = robot.jacob0(q);
    
    
    lambda =0.1;
    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';

    
    %dq = inv(J)*dx
    dq = inv(Jinv_dls)*dx
    q = q +(dq*dt)'
    
    
    % Update plot
    robot.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
      
