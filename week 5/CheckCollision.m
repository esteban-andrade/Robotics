function[bool]=  CheckCollision(robot,sphereCentre,radius)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
pause(0.01);
tr = robot.fkine(robot.getpos);
endEffectorToCentreDist = sqrt(sum((sphereCentre-tr(1:3,4)').^2));

if endEffectorToCentreDist <=radius
    disp('Collision');
    bool=true;
else
    disp(['Safe: End Effector tp Sphere centre distance : ',num2str(endEffectorToCentreDist)]);
    bool = false;
end
end

