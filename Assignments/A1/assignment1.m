%% UR3 model

clear all
clc
clf

set(0,'DefaultFigureWindowStyle','docked');
view(3);
UR3A = UR3('goku');
%hold on
UR5A = LinearUR5(false);%change to UR5 later
axis image
baseATransform = transl(0.65,0,0);
baseBTransform = transl(-0.75,-0.35,0)*trotx(pi/2);%*troty(pi/2) this is to give it the same  orientation as the UR3;
centreA = baseATransform(:,4);
centreB= baseBTransform(:,4);
centre = (centreA+centreB)/2;
UR3A.model.base = baseATransform;
UR5A.model.base = baseBTransform;
%UR3 Joint angles zero state
q = zeros(1,6);
p = zeros(1,7);
animate(UR3A.model, q);
hold on
animate(UR5A.model, p);
% UR3A.model.teach();
% UR3B.model.teach();
%% Get Model Volume Matrix
%clf
clc
volumeRobot=1; %if 1 UR3, if 2 UR5 ,IF both 3
degrees = 60;

figure('Name','Point Clouds Robot')
if volumeRobot==1
    UR3A.DrawVolumeArm(degrees);
    volumeUR3=UR3A.volume;
elseif volumeRobot==2
    volumeUR5 = UR5Volume(UR5A,degrees);
    
elseif volumeRobot==3
    UR3A.DrawVolumeArm(degrees);
    volumeUR3=UR3A.volume;
    hold on
    volumeUR5 = UR5Volume(UR5A,degrees);
    
end


figure ('Name','UR3 Volume');
UR3A.getReach();
UR3A.plotArmVolume();
disp(['Maximum Reach in X&Y horizontally: ',num2str(UR3A.transveralReach),' m']);
disp(['Maximum Reach in Z vertically: ',num2str(UR3A.verticalReach),' m']);
disp(['Calculated Arc radius of workSpace: ',num2str(UR3A.arcRadius),' m']);
disp(['Calculated WorkSpace Volume: ',num2str(UR3A.computedVolume),' m3']);


%% Loading Enviroment

disp('Loading Models and Environment');

middlePointInBetweenRobot= middlePointInBetween(baseATransform,baseBTransform);
tablePosition =[middlePointInBetweenRobot(1)-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)-0.356-centre(3)];
tableOrientation =[0,0,0];
Table = RecontructObject('table3.ply',tableOrientation,tablePosition);

floorPosition = [middlePointInBetweenRobot(1)+0.7-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)-centre(3)];
floorOrientation =[0,pi/2,0];
Floor = RecontructObjectNonRGB('floor2.ply',floorOrientation,floorPosition);
%BRICK 1 to 4 for UR3
brick1Position = [0.5,-0.5,-0.06];
brick1Orientation = [pi,0,0];
brick1 = RecontructObject('Brick.ply',brick1Orientation,brick1Position);
brick1Destiny = transl(0,-0.2,0.06)*rpy2tr(brick1Orientation);

brick2Position = [0.5,0.5,-0.06];
brick2Orientation = [pi,0,0];
brick2 = RecontructObject('Brick.ply',brick2Orientation,brick2Position);
brick2Destiny = transl(0,0.2,0.06)*rpy2tr(brick2Orientation);

brick3Position = [0.75,0.5,-0.06];
brick3Orientation = [pi,0,0];
brick3 = RecontructObject('Brick.ply',brick3Orientation,brick3Position);
brick3Destiny = transl(0,0,0.12)*rpy2tr(brick3Orientation);

brick4Position = [0.75,-0.5,-0.06];
brick4Orientation = [pi,0,0];
brick4 = RecontructObject('Brick.ply',brick4Orientation,brick4Position);
brick4Destiny = transl(0,-0.2,0.18)*rpy2tr(brick4Orientation);

%BRICK 5 to 9 for UR5

brick5Position = [-1,-0.75,-0.06];
brick5Orientation = [pi,0,0];
brick5 = RecontructObject('Brick.ply',brick5Orientation,brick5Position);
brick5Destiny = transl(0,0,0.06)*rpy2tr(brick5Orientation);

brick6Position = [-1,0.75,-0.06];
brick6Orientation = [pi,0,0];
brick6 = RecontructObject('Brick.ply',brick6Orientation,brick6Position);
brick6Destiny = transl(0,0.2,0.12)*rpy2tr(brick6Orientation);

brick7Position = [-1.5,-0.75,-0.06];
brick7Orientation = [pi,0,0];
brick7 = RecontructObject('Brick.ply',brick7Orientation,brick7Position);
brick7Destiny = transl(0,-0.2,0.12)*rpy2tr(brick7Orientation);

brick8Position = [-1.5,0.75,-0.06];
brick8Orientation = [pi,0,0];
brick8 = RecontructObject('Brick.ply',brick8Orientation,brick8Position);
brick8Destiny = transl(0,0,0.18)*rpy2tr(brick8Orientation);

brick9Position = [-1.5,0,-0.06];
brick9Orientation = [pi,0,0];
brick9 = RecontructObject('Brick.ply',brick9Orientation,brick9Position);
brick9Destiny = transl(0,0.2,0.18)*rpy2tr(brick9Orientation);

BobPosition = [middlePointInBetweenRobot(1)-centre(1),middlePointInBetweenRobot(2)+3-centre(2),middlePointInBetweenRobot(3)-centre(3)];
BobOrientation = [0,0,0];
Bob = RecontructObjectNonRGB('full_body.ply',BobOrientation,BobPosition);

fence1Position= [middlePointInBetweenRobot(1)+2.5-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)+0.3-centre(3)];
fence1Orientation = [0,0,pi/2];
fence1 = RecontructObject('fence.ply',fence1Orientation,fence1Position);

fence2Position= [middlePointInBetweenRobot(1)-2.5-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)+0.3-centre(3)];
fence2Orientation = [0,0,pi/2];
fence2 = RecontructObject('fence.ply',fence2Orientation,fence2Position);

fence3Position= [middlePointInBetweenRobot(1)+2.5-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)+0.3-centre(3)];
fence3Orientation = [0,0,0];
fence3 = RecontructObject('fence.ply',fence3Orientation,fence3Position);

fence4Position= [middlePointInBetweenRobot(1)-2.5-centre(1),middlePointInBetweenRobot(2)-centre(2),middlePointInBetweenRobot(3)+0.3-centre(3)];
fence4Orientation = [0,0,0];
fence4 = RecontructObject('fence.ply',fence4Orientation,fence4Position);

eButtonPosition = [middlePointInBetweenRobot(1)+1.7-centre(1),middlePointInBetweenRobot(2)+0.7-centre(2),middlePointInBetweenRobot(3)+0.1-centre(3)];
eButtonOrientation = [0,0,0];
eButton = RecontructObject('emergency.ply',eButtonOrientation,eButtonPosition);

warning1Position = [middlePointInBetweenRobot(1)-centre(1),middlePointInBetweenRobot(2)-2.6-centre(2),middlePointInBetweenRobot(3)+0.7-centre(3)];
warning1Orientation = [0,0,0];
warning1 = RecontructObject('warning.ply',warning1Orientation,warning1Position);

warning2Position = [middlePointInBetweenRobot(1)-centre(1),middlePointInBetweenRobot(2)-2.6-centre(2),middlePointInBetweenRobot(3)+0.7-centre(3)];
warning2Orientation = [0,0,pi];
warning2 = RecontructObject('warning.ply',warning2Orientation,warning2Position);

animate(UR3A.model, q);
hold on
animate(UR5A.model, p);
disp('Environment Imported');

%% Kinematics
steps=60;
mode =2; % Mode 1 for single arm movement, Mode 2 double arm movement (Order UR3 to UR5)
brickCollected = false;
brickDeposited=false;

qUR3still = [0 0 0 0 0 0];
qUR5still = [0 0 0 0 0 0 0];

if mode ==1
    
    q1_1=UR3A.model.getpos();
    [q2_1,err2_1,exitflag_1]=UR3A.model.ikcon(brick1.pose,q1_1);
    qmatrix_1 = robotMotion.interpolateJointAnglesUR3(q1_1,q2_1,steps);
    robotMotion.motion(qmatrix_1,UR3A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_1 = qmatrix_1(end,:);
        [q4_1,err4_1,exitflag4_1]=UR3A.model.ikcon(brick1Destiny,q3_1);
        qmatrixDes_1 = robotMotion.interpolateJointAnglesUR3(q3_1,q4_1,steps);
        robotMotion.objectMotion(qmatrixDes_1,UR3A.model,brick1,brick1Destiny);
        
        brickDeposited=true;
        if brickDeposited==true
            
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            robotMotion.motion(restoreQMatrixUR3,UR3A.model);
            brickCollected = false;
            brickDeposited=false;
        end
    end
    
    q1_2=UR5A.model.getpos();
    [q2_2,err2_2,exitflag_2]=UR5A.model.ikcon(brick5.pose,q1_2);
    qmatrix_2 = robotMotion.interpolateJointAnglesLinearUR5(q1_2,q2_2,steps);
    robotMotion.motion(qmatrix_2,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_2 = qmatrix_2(end,:);
        [q4_2,err4_2,exitflag4_2]=UR5A.model.ikcon(brick5Destiny,q3_2);
        qmatrixDes_2 = robotMotion.interpolateJointAnglesLinearUR5(q3_2,q4_2,steps);
        
        robotMotion.objectMotion(qmatrixDes_2,UR5A.model,brick5,brick5Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
        end
    end
    
    q1_3=UR3A.model.getpos();
    [q2_3,err2_3,exitflag_3]=UR3A.model.ikcon(brick2.pose,q1_3);
    qmatrix_3 = robotMotion.interpolateJointAnglesUR3(q1_3,q2_3,steps);
    robotMotion.motion(qmatrix_3,UR3A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_3 = qmatrix_3(end,:);
        [q4_3,err4_3,exitflag4_3]=UR3A.model.ikcon(brick2Destiny,q3_3);
        qmatrixDes_3 = robotMotion.interpolateJointAnglesUR3(q3_3,q4_3,steps);
        robotMotion.objectMotion(qmatrixDes_3,UR3A.model,brick2,brick2Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            robotMotion.motion(restoreQMatrixUR3,UR3A.model);
            brickCollected = false;
            brickDeposited=false;
        end
    end
    
    q1_4=UR5A.model.getpos();
    [q2_4,err2_4,exitflag_4]=UR5A.model.ikcon(brick6.pose,q1_4);
    qmatrix_4 = robotMotion.interpolateJointAnglesLinearUR5(q1_4,q2_4,steps);
    
    robotMotion.motion(qmatrix_4,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_4 = qmatrix_4(end,:);
        [q4_4,err4_4,exitflag4_4]=UR5A.model.ikcon(brick6Destiny,q3_4);
        qmatrixDes_4 = robotMotion.interpolateJointAnglesLinearUR5(q3_4,q4_4,steps);
        robotMotion.objectMotion(qmatrixDes_4,UR5A.model,brick6,brick6Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
            
        end
    end
    
    q1_5=UR3A.model.getpos();
    [q2_5,err2_5,exitflag_5]=UR3A.model.ikcon(brick3.pose,q1_5);
    qmatrix_5 = robotMotion.interpolateJointAnglesUR3(q1_5,q2_5,steps);
    robotMotion.motion(qmatrix_5,UR3A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_5 = qmatrix_5(end,:);
        [q4_5,err4_5,exitflag4_5]=UR3A.model.ikcon(brick3Destiny,q3_5);
        qmatrixDes_5 = robotMotion.interpolateJointAnglesUR3(q3_5,q4_5,steps);
        robotMotion.objectMotion(qmatrixDes_5,UR3A.model,brick3,brick3Destiny);
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            robotMotion.motion(restoreQMatrixUR3,UR3A.model);
            brickCollected = false;
            brickDeposited=false;
        end
    end
    
    q1_6=UR5A.model.getpos();
    [q2_6,err2_6,exitflag_6]=UR5A.model.ikcon(brick7.pose,q1_6);
    qmatrix_6 = robotMotion.interpolateJointAnglesLinearUR5(q1_6,q2_6,steps);
    
    robotMotion.motion(qmatrix_6,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_6 = qmatrix_6(end,:);
        [q4_6,err4_6,exitflag4_6]=UR5A.model.ikcon(brick7Destiny,q3_6);
        qmatrixDes_6 = robotMotion.interpolateJointAnglesLinearUR5(q3_6,q4_6,steps);
        robotMotion.objectMotion(qmatrixDes_6,UR5A.model,brick7,brick7Destiny);
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
            
        end
    end
    
    q1_7=UR3A.model.getpos();
    [q2_7,err2_7,exitflag_7]=UR3A.model.ikcon(brick4.pose,q1_7);
    qmatrix_7 = robotMotion.interpolateJointAnglesUR3(q1_7,q2_7,steps);
    robotMotion.motion(qmatrix_7,UR3A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_7 = qmatrix_7(end,:);
        [q4_7,err4_7,exitflag4_7]=UR3A.model.ikcon(brick4Destiny,q3_7);
        qmatrixDes_7 = robotMotion.interpolateJointAnglesUR3(q3_7,q4_7,steps);
        robotMotion.objectMotion(qmatrixDes_7,UR3A.model,brick4,brick4Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            robotMotion.motion(restoreQMatrixUR3,UR3A.model);
            brickCollected = false;
            brickDeposited=false;
        end
    end
    
    q1_8=UR5A.model.getpos();
    [q2_8,err2_8,exitflag_8]=UR5A.model.ikcon(brick8.pose,q1_8);
    qmatrix_8 = robotMotion.interpolateJointAnglesLinearUR5(q1_8,q2_8,steps);
    
    robotMotion.motion(qmatrix_8,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_8 = qmatrix_8(end,:);
        [q4_8,err4_8,exitflag4_8]=UR5A.model.ikcon(brick8Destiny,q3_8);
        qmatrixDes_8 = robotMotion.interpolateJointAnglesLinearUR5(q3_8,q4_8,steps);
        
        robotMotion.objectMotion(qmatrixDes_8,UR5A.model,brick8,brick8Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
            
        end
    end
    
    q1_9=UR5A.model.getpos();
    [q2_9,err2_9,exitflag_9]=UR5A.model.ikcon(brick9.pose,q1_9);
    qmatrix_9 = robotMotion.interpolateJointAnglesLinearUR5(q1_9,q2_9,steps);
    
    robotMotion.motion(qmatrix_9,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_9 = qmatrix_9(end,:);
        [q4_9,err4_9,exitflag4_9]=UR5A.model.ikcon(brick9Destiny,q3_9);
        qmatrixDes_9 = robotMotion.interpolateJointAnglesLinearUR5(q3_9,q4_9,steps);
        
        robotMotion.objectMotion(qmatrixDes_8,UR5A.model,brick9,brick9Destiny);
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
            
        end
    end
    
elseif mode ==2
    
    q1_1=UR3A.model.getpos();
    [q2_1,err2_1,exitflag_1]=UR3A.model.ikcon(brick1.pose,q1_1);
    qmatrix_1 = robotMotion.interpolateJointAnglesUR3(q1_1,q2_1,steps);
    
    q1_2=UR5A.model.getpos();
    [q2_2,err2_2,exitflag_2]=UR5A.model.ikcon(brick5.pose,q1_2);
    qmatrix_2 = robotMotion.interpolateJointAnglesLinearUR5(q1_2,q2_2,steps);
    
    robotMotion.doubleMotion(qmatrix_1,UR3A.model,qmatrix_2,UR5A.model)
    brickCollected=true;
    
    if brickCollected ==true
        q3_1 = qmatrix_1(end,:);
        [q4_1,err4_1,exitflag4_1]=UR3A.model.ikcon(brick1Destiny,q3_1);
        qmatrixDes_1 = robotMotion.interpolateJointAnglesUR3(q3_1,q4_1,steps);
        q3_2 = qmatrix_2(end,:);
        [q4_2,err4_2,exitflag4_2]=UR5A.model.ikcon(brick5Destiny,q3_2);
        qmatrixDes_2 = robotMotion.interpolateJointAnglesLinearUR5(q3_2,q4_2,steps);
        
        robotMotion.doubleObjectMotion(qmatrixDes_1,UR3A.model,brick1,brick1Destiny,qmatrixDes_2,UR5A.model,brick5,brick5Destiny);
        brickDeposited=true;
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.doubleMotion(restoreQMatrixUR3,UR3A.model,restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
        end
        
    end
    
    
    q1_3=UR3A.model.getpos();
    [q2_3,err2_3,exitflag_3]=UR3A.model.ikcon(brick2.pose,q1_3);
    qmatrix_3 = robotMotion.interpolateJointAnglesUR3(q1_3,q2_3,steps);
    
    q1_4=UR5A.model.getpos();
    [q2_4,err2_4,exitflag_4]=UR5A.model.ikcon(brick6.pose,q1_4);
    qmatrix_4 = robotMotion.interpolateJointAnglesLinearUR5(q1_4,q2_4,steps);
    
    robotMotion.doubleMotion(qmatrix_3,UR3A.model,qmatrix_4,UR5A.model)
    brickCollected=true;
    
    if brickCollected ==true
        q3_3 = qmatrix_3(end,:);
        [q4_3,err4_3,exitflag4_3]=UR3A.model.ikcon(brick2Destiny,q3_3);
        qmatrixDes_3 = robotMotion.interpolateJointAnglesUR3(q3_3,q4_3,steps);
        q3_4 = qmatrix_4(end,:);
        [q4_4,err4_4,exitflag4_4]=UR5A.model.ikcon(brick6Destiny,q3_4);
        qmatrixDes_4 = robotMotion.interpolateJointAnglesLinearUR5(q3_4,q4_4,steps);
        
        robotMotion.doubleObjectMotion(qmatrixDes_3,UR3A.model,brick2,brick2Destiny,qmatrixDes_4,UR5A.model,brick6,brick6Destiny);
        brickDeposited=true;
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.doubleMotion(restoreQMatrixUR3,UR3A.model,restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
        end
        
    end
    
    q1_5=UR3A.model.getpos();
    [q2_5,err2_5,exitflag_5]=UR3A.model.ikcon(brick3.pose,q1_5);
    qmatrix_5 = robotMotion.interpolateJointAnglesUR3(q1_5,q2_5,steps);
    
    q1_6=UR5A.model.getpos();
    [q2_6,err2_6,exitflag_6]=UR5A.model.ikcon(brick7.pose,q1_6);
    qmatrix_6 = robotMotion.interpolateJointAnglesLinearUR5(q1_6,q2_6,steps);
    
    robotMotion.doubleMotion(qmatrix_5,UR3A.model,qmatrix_6,UR5A.model)
    brickCollected=true;
    
    if brickCollected ==true
        q3_5 = qmatrix_5(end,:);
        [q4_5,err4_5,exitflag4_5]=UR3A.model.ikcon(brick3Destiny,q3_5);
        qmatrixDes_5 = robotMotion.interpolateJointAnglesUR3(q3_5,q4_5,steps);
        q3_6 = qmatrix_6(end,:);
        [q4_6,err4_6,exitflag4_6]=UR5A.model.ikcon(brick7Destiny,q3_6);
        qmatrixDes_6 = robotMotion.interpolateJointAnglesLinearUR5(q3_6,q4_6,steps);
        
        robotMotion.doubleObjectMotion(qmatrixDes_5,UR3A.model,brick3,brick3Destiny,qmatrixDes_6,UR5A.model,brick7,brick7Destiny);
        brickDeposited=true;
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.doubleMotion(restoreQMatrixUR3,UR3A.model,restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
        end
        
    end
    
    q1_7=UR3A.model.getpos();
    [q2_7,err2_7,exitflag_7]=UR3A.model.ikcon(brick4.pose,q1_7);
    qmatrix_7 = robotMotion.interpolateJointAnglesUR3(q1_7,q2_7,steps);
    
    q1_8=UR5A.model.getpos();
    [q2_8,err2_8,exitflag_8]=UR5A.model.ikcon(brick8.pose,q1_8);
    qmatrix_8 = robotMotion.interpolateJointAnglesLinearUR5(q1_8,q2_8,steps);
    
    robotMotion.doubleMotion(qmatrix_7,UR3A.model,qmatrix_8,UR5A.model)
    brickCollected=true;
    
    if brickCollected ==true
        q3_7 = qmatrix_7(end,:);
        [q4_7,err4_7,exitflag4_7]=UR3A.model.ikcon(brick4Destiny,q3_7);
        qmatrixDes_7 = robotMotion.interpolateJointAnglesUR3(q3_7,q4_7,steps);
        q3_8 = qmatrix_8(end,:);
        [q4_8,err4_8,exitflag4_8]=UR5A.model.ikcon(brick8Destiny,q3_8);
        qmatrixDes_8 = robotMotion.interpolateJointAnglesLinearUR5(q3_8,q4_8,steps);
        
        robotMotion.doubleObjectMotion(qmatrixDes_7,UR3A.model,brick4,brick4Destiny,qmatrixDes_8,UR5A.model,brick8,brick8Destiny);
        brickDeposited=true;
        if brickDeposited==true
            restoreQMatrixUR3 = robotMotion.interpolateJointAnglesUR3(UR3A.model.getpos,qUR3still,steps);
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.doubleMotion(restoreQMatrixUR3,UR3A.model,restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
        end
        
    end
    
    q1_9=UR5A.model.getpos();
    [q2_9,err2_9,exitflag_9]=UR5A.model.ikcon(brick9.pose,q1_9);
    qmatrix_9 = robotMotion.interpolateJointAnglesLinearUR5(q1_9,q2_9,steps);
    
    robotMotion.motion(qmatrix_9,UR5A.model);
    brickCollected=true;
    
    if brickCollected ==true
        q3_9 = qmatrix_9(end,:);
        [q4_9,err4_9,exitflag4_9]=UR5A.model.ikcon(brick9Destiny,q3_9);
        qmatrixDes_9 = robotMotion.interpolateJointAnglesLinearUR5(q3_9,q4_9,steps);
        
        robotMotion.objectMotion(qmatrixDes_8,UR5A.model,brick9,brick9Destiny);
        
        
        brickDeposited=true;
        
        if brickDeposited==true
            restoreQMatrixUR5 =robotMotion.interpolateJointAnglesLinearUR5(UR5A.model.getpos,qUR5still,steps);
            robotMotion.motion(restoreQMatrixUR5,UR5A.model);
            brickCollected = false;
            brickDeposited=false;
            
        end
    end
    
    
end


%% ROS BAG

clc
bag=rosbag('2018-03-20-18-34-46 .bag');
topics = select(bag,'Topic','/joint_states');
messages = readMessages(topics);
resolution = 10;

dataMode =1;% mode will be based for normal given data set or we take the transpose of that

if dataMode==1
    for i = 1:(size(messages,1)/resolution) %reduces the size of the matrix as the step resolution is too high and unnecessary
        if i == 1
            qMatrixBag(i,:) = (messages{i}.Position); %assigns a qMatrix with the joint states from the bag file
        else
            qMatrixBag(i,:) = (messages{i*resolution}.Position); %assigns a qMatrix with the joint states from the bag file
            
        end
    end
    
elseif dataMode==2
    if i == 1
        qMatrixBag(i,:) = (messages{i}.Position)'; %assigns a qMatrix with the joint states from the bag file
    else
        qMatrixBag(i,:) = (messages{i*resolution}.Position)'; %assigns a qMatrix with the joint states from the bag file
        
    end
end


for i = 1:size(qMatrixBag,1)
    UR3A.model.animate(qMatrixBag(i,:));
    text = ['Step number: ', num2str(i)];
    disp(text); %shows that it is running even though it is quite slow
    drawnow() %updates the figure
    pause(0.01);
end
