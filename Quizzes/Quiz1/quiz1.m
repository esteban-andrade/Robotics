%% Quadcopter

% y and z need to be inverted
vec1 = [0, 0, 8];
vec2 = [8, -8, -12];
vec3 = [-5, -3, 2];

trStart = eye(4);
trNext = trStart * transl(vec1);
tranimate(trStart, trNext, 'fps', 25);
trStart = trNext;

trNext = trStart * transl(vec2);
tranimate(trStart, trNext, 'fps', 25);
trStart = trNext;

trNext = trStart * transl(vec3);
tranimate(trStart, trNext, 'fps', 25);
trStart = trNext

%% Create new Puma Robot
clear
mdl_puma560
p560.teach

%% Puma Robot Calculations
limits = p560.qlim;
limitsMin = limits(:,1);
limitsMax = limits(:,2);

pos = transpose(p560.getpos);

diffMin = abs(limitsMin - pos);
diffMax = abs(limitsMax - pos);

closestJointLimit = min([diffMin diffMax], [], 2)

endPose = p560.fkine(pos);
xyz_mm = transl(1000 * endPose)
rpy = tr2rpy(endPose, 'deg')


%% Octagon Track

iterations = 4;
start = se3(se2(100, 0, 0));

sideLength = 90;
turnAngle = 45 * (pi / 180);
motion = se3(se2(sideLength, 0, turnAngle));

for i = 1:iterations
    nextPos = start * motion;
    start = nextPos;
end

xyz = transl(nextPos)
rpy = tr2rpy(nextPos, 'deg')