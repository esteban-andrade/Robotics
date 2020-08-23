function [output] = middlePointInBetween(object1,object2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
x = (object1(1,4) + object2(1,4))/2;
y = (object1(2,4) + object2(2,4))/2;
z = (object1(3,4) + object2(3,4))/2;

output=[x,y,z];
end

