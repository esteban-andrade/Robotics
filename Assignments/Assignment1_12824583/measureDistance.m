function [distance] = measureDistance(object1, object2)
%Function to get the measured distance between 2 object positions in space

distance = sqrt((object1(1, 4)-object2(1, 4))^2+(object1(2, 4) - object2(2, 4))^2+(object1(3, 4) - object2(3, 4))^2);
end
