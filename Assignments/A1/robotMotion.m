classdef robotMotion
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        endEffector;
    end
    
    methods(Static)
        
        function motion(qMatrix,robot)
            
             logData =['transforms-data.mat'];
            
            for step =1:size(qMatrix,1)
                %animate(robot,qMatrix(step,:));
                robot.animate(qMatrix(step,:));
                endEffector=robot.fkine(qMatrix(step,:)); %#ok<NASGU>
                
                
                
                            %Logs the data in the file premade earlier
                            if ( step == size(qMatrix,1)) %if this is the last step of the arm
                
                                transformsData.name= robot.name;
                                transformsData.pose= robot.fkine(qMatrix(step,:));
                
                                text = [robot.name ' End-Effector Transform '];
                                disp(text);
                                disp(transformsData.pose);
                             
                
                                if isfile(logData)
                                    %if the file exists then add to the mat file
                                    a=load(logData);
                
                                    transformsData=[a.transformsData; transformsData]; %adds data to the end of the mat file
                
                                    save(logData,'transformsData');
                                else
                                    %if the file does not exist, save
                                    save(logData,'transformsData');
                                end
                
                            end
                
                
                
                
                drawnow()
                
            end
        end
        %%
        function qMatrixUR3= interpolateJointAnglesUR3(q1,q2,steps)
            s = lspb(0,1,steps);                                             	% First, create the scalar function
            qMatrixUR3 = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR3(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
        end
        %%
        function qMatrixUR5 = interpolateJointAnglesLinearUR5(q1,q2,steps)
            s = lspb(0,1,steps);                                             	% First, create the scalar function
            qMatrixUR5 = nan(steps,7);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR5(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
            
            %qMatrixUR5 = jtraj(q1,q2,steps);
        end
        
        
    end
end

