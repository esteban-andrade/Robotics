function [distance] = measureDistance(object1,object2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
distance = sqrt((robot.base(1,4)-object2(1,4))^2+(object1(2,4)-object2(2,4))^2+(object1(3,4)-object2(3,4))^2);
end

