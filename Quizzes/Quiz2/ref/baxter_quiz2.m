%% Create baxter model
clc
clf
clear all

mdl_baxter()

left.teach()
hold on
right.teach()

%% Alter base transform

x =0;
y =0;
z =0;

left.base = transl(left.base(1,4)+x, left.base(2,4)+y, left.base(3,4)+z)*rpy2tr(0, 0, pi/4);
right.base = transl(right.base(1,4)+x, right.base(2,4)+y, right.base(3,4)+z)*rpy2tr(0, 0, -pi/4);

%new xyz locations from offset added
xyzLeftBase = left.base(1:3,4);
xyzRightZBase = right.base(1:3,4);
