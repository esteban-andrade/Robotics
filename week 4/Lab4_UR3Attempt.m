 %% UR3 model

clear all
clc
clf

set(0,'DefaultFigureWindowStyle','docked');
view(3);
UR3A = UR3;
UR3B = UR3;

baseATransform = transl(-0.5, 0.5,0);
baseBTransform = transl(0.5, -0.5,0);

UR3A.model.base = baseATransform;
UR3B.model.base = baseBTransform;
%UR3 Joint angles zero state
q = zeros(1,6);
animate(UR3A.model, q);
animate(UR3B.model, q);
%UR3A.model.teach();
%UR3B.model.teach();

