clear all
clc

mdl_puma560
fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]))) == fkine(p560,[pi/4,0,0,0,0,0])
fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0])) == fkine(p560,[pi/4,0,0,0,0,0])
% fkine(p560,ikine(p560,fkine(p560,[pi/4,0,0,0,0,0]))) == fkine(p560,[pi/4,0,0,0,0,0])
% fkine(p560,ikine(p560,fkine(p560,[-pi/4,0,0,0,0,0]),[pi/4,0,0,0,0,0])) == fkine(p560,[-pi/4,0,0,0,0,0])