classdef robotMotion %The class was develop for arm motion for different robots.
    %Also to simulate the movement of object once the end effector reaches
    %that position

    properties
        endEffector;
    end

    methods (Static)

        function motion(qMatrix, robot) %pass Q matrix and corresponding robot

            logData = ['transforms-data.mat']; % log data into a mat file

            for step = 1:size(qMatrix, 1) % iterate between rows of Q matrix

                %animate(robot,qMatrix(step,:));
                robot.animate(qMatrix(step, :));
                endEffector = robot.fkine(qMatrix(step, :)); %#ok<NASGU>

                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arm

                    transformsData.name = robot.name;
                    transformsData.pose = robot.fkine(qMatrix(step, :));
                    text = [robot.name, ' End-Effector Transform '];
                    disp(text);
                    disp(transformsData.pose);

                    if isfile(logData)
                        %check if the file exists then add to the mat file
                        a = load(logData);
                        transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file
                        save(logData, 'transformsData');
                    else
                        %if the file does not exist, save
                        save(logData, 'transformsData');
                    end

                end
                drawnow()
            end
        end

        %% Function to perform motion of 2 robots

        function doubleMotion(qMatrixA, robot1, qMatrixB, robot2) %pass both q matrices for corresponding robots

            %logs data into 2 Mat files
            logData1 = ['transforms-data-1.mat'];
            logData2 = ['transforms-data-2.mat'];

            qMatrix = [qMatrixA, qMatrixB]; %concatonate matrices

            for step = 1:size(qMatrix, 1)
                %animate(robot,qMatrix(step,:));
                robot1.animate(qMatrix(step, 1:6));
                endEffector1 = robot1.fkine(qMatrix(step, 1:6)); %#ok<NASGU>
                robot2.animate(qMatrix(step, 7:end));
                endEffector2 = robot2.fkine(qMatrix(step, 7:end)); %#ok<NASGU>


                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arms

                    transformsData1.name = robot1.name;
                    transformsData1.pose = robot1.fkine(qMatrix(step, 1:6));

                    text1 = [robot1.name, ' End-Effector Transform '];
                    disp(text1);
                    disp(transformsData1.pose);

                    transformsData2.name = robot2.name;
                    transformsData2.pose = robot2.fkine(qMatrix(step, 7:end));

                    text2 = [robot2.name, ' End-Effector Transform '];
                    disp(text2);
                    disp(transformsData2.pose);


                    if isfile(logData1)
                        %check if the file exists then add to the mat file
                        a = load(logData1);

                        transformsData1 = [a.transformsData1; transformsData1]; %adds data to the end of the mat file

                        save(logData1, 'transformsData1');
                    else
                        %check if the file exists then add to the mat file
                        save(logData1, 'transformsData1');
                    end


                    if isfile(logData2)
                        %check if the file exists then add to the mat file
                        b = load(logData2);

                        transformsData2 = [b.transformsData2; transformsData2]; %adds data to the end of the mat file

                        save(logData2, 'transformsData2');
                    else
                        %check if the file exists then add to the mat file
                        save(logData2, 'transformsData2');
                    end

                end
                drawnow()
            end
        end

        %% Function to simulate object motion along with the end effector

        function objectMotion(qMatrix, robot, object, objectDestiny) % pass qmatrix, robot and object details

            logData = ['transforms-data.mat']; % log data into a mat file

            for step = 1:size(qMatrix, 1)
                %animate(robot,qMatrix(step,:));
                robot.animate(qMatrix(step, :));
                endEffector = robot.fkine(qMatrix(step, :)); %#ok<NASGU>


                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arm

                    transformsData.name = robot.name;
                    transformsData.pose = robot.fkine(qMatrix(step, :));

                    text = [robot.name, ' End-Effector Transform '];
                    disp(text);
                    disp(transformsData.pose);


                    if isfile(logData)
                        %check if the file exists then add to the mat file
                        a = load(logData);

                        transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file

                        save(logData, 'transformsData');
                    else
                        %save data
                        save(logData, 'transformsData');
                    end

                end

                %  give end effector pose to object and update object
                %  parameters
                object.pose = endEffector;
                newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
                object.mesh.Vertices = newPoints(:, 1:3);
                drawnow()

            end

            % plot object at the final position
            object.pose = objectDestiny;
            newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
            object.mesh.Vertices = newPoints(:, 1:3);
            drawnow()
        end

        %% Function to perform double robot motion for double object animation

        function doubleObjectMotion(qMatrixA, robot1, object1, object1Destiny, qMatrixB, robot2, object2, object2Destiny)

            %logs data into 2 mat files
            logData1 = ['transforms-data-1.mat'];
            logData2 = ['transforms-data-2.mat'];

            qMatrix = [qMatrixA, qMatrixB]; %concatonate matrices

            for step = 1:size(qMatrix, 1)
                %animate(robot,qMatrix(step,:));
                robot1.animate(qMatrix(step, 1:6));
                endEffector1 = robot1.fkine(qMatrix(step, 1:6)); %#ok<NASGU>
                robot2.animate(qMatrix(step, 7:end));
                endEffector2 = robot2.fkine(qMatrix(step, 7:end)); %#ok<NASGU>

                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arms

                    transformsData1.name = robot1.name;
                    transformsData1.pose = robot1.fkine(qMatrix(step, 1:6));

                    text1 = [robot1.name, ' End-Effector Transform '];
                    disp(text1);
                    disp(transformsData1.pose);

                    transformsData2.name = robot2.name;
                    transformsData2.pose = robot2.fkine(qMatrix(step, 7:end));

                    text2 = [robot2.name, ' End-Effector Transform '];
                    disp(text2);
                    disp(transformsData2.pose);


                    if isfile(logData1)
                        %check if the file exists then add to the mat file
                        a = load(logData1);

                        transformsData1 = [a.transformsData1; transformsData1]; %adds data to the end of the mat file
                        %save log
                        save(logData1, 'transformsData1');
                    else
                        %save log
                        save(logData1, 'transformsData1');
                    end


                    if isfile(logData2)
                        %check if the file exists then add to the mat file
                        b = load(logData2);

                        transformsData2 = [b.transformsData2; transformsData2]; %adds data to the end of the mat file
                        %save log
                        save(logData2, 'transformsData2');
                    else
                        %save log
                        save(logData2, 'transformsData2');
                    end

                end
                %  give end effector pose to object and update object  parameters
                object1.pose = endEffector1;
                newPoints1 = [object1.pose * [object1.verts, ones(object1.vertexCount, 1)]']'; %#ok<NBRAK>
                object1.mesh.Vertices = newPoints1(:, 1:3);
                %  give end effector pose to object and update object  parameters
                object2.pose = endEffector2;
                newPoints2 = [object2.pose * [object2.verts, ones(object2.vertexCount, 1)]']'; %#ok<NBRAK>
                object2.mesh.Vertices = newPoints2(:, 1:3);

                drawnow()

            end
            % plot object at the final position
            object1.pose = object1Destiny;
            newPoints1 = [object1.pose * [object1.verts, ones(object1.vertexCount, 1)]']'; %#ok<NBRAK>
            object1.mesh.Vertices = newPoints1(:, 1:3);

            % plot object at the final position
            object2.pose = object2Destiny;
            newPoints2 = [object2.pose * [object2.verts, ones(object2.vertexCount, 1)]']'; %#ok<NBRAK>
            object2.mesh.Vertices = newPoints2(:, 1:3);

            drawnow()

        end

        %% Function to perform double motion for 2 robots using a single Q matrix


        function doubleMotionSingleMatrix(qMatrix, robot1, robot2)

            %logs data into 2 mat files
            logData1 = ['transforms-data-1.mat'];
            logData2 = ['transforms-data-2.mat'];


            for step = 1:size(qMatrix, 1)
                %animate(robot,qMatrix(step,:));
                robot1.animate(qMatrix(step, 1:6));
                endEffector1 = robot1.fkine(qMatrix(step, 1:6)); %#ok<NASGU>
                robot2.animate(qMatrix(step, 7:end));
                endEffector2 = robot2.fkine(qMatrix(step, 7:end)); %#ok<NASGU>


                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arms

                    transformsData1.name = robot1.name;
                    transformsData1.pose = robot1.fkine(qMatrix(step, 1:6));

                    text1 = [robot1.name, ' End-Effector Transform '];
                    disp(text1);
                    disp(transformsData1.pose);

                    transformsData2.name = robot2.name;
                    transformsData2.pose = robot2.fkine(qMatrix(step, 7:end));

                    text2 = [robot2.name, ' End-Effector Transform '];
                    disp(text2);
                    disp(transformsData2.pose);


                    if isfile(logData1)
                        %check if the file exists then add to the mat file
                        a = load(logData1);
                        transformsData1 = [a.transformsData1; transformsData1]; %adds data to the end of the mat file
                        save(logData1, 'transformsData1');
                    else
                        %save log
                        save(logData1, 'transformsData1');
                    end


                    if isfile(logData2)
                        %check if the file exists then add to the mat file
                        b = load(logData2);
                        transformsData2 = [b.transformsData2; transformsData2]; %adds data to the end of the mat file
                        save(logData2, 'transformsData2');
                    else
                        %save log
                        save(logData2, 'transformsData2');
                    end
                end
                drawnow()

            end
        end

        %% Function used to Perform object motion passing single combined object parameter

        function objectMotionCombined(qMatrix, robot, object)

            % log data into a mat file

            logData = ['transforms-data.mat'];

            for step = 1:size(qMatrix, 1)
                %animate(robot,qMatrix(step,:));
                robot.animate(qMatrix(step, :));
                endEffector = robot.fkine(qMatrix(step, :)); %#ok<NASGU>


                %Logs the data in the file premade earlier
                if (step == size(qMatrix, 1)) %if this is the last step of the arm

                    transformsData.name = robot.name;
                    transformsData.pose = robot.fkine(qMatrix(step, :));

                    text = [robot.name, ' End-Effector Transform '];
                    disp(text);
                    disp(transformsData.pose);


                    if isfile(logData)
                        %check if the file exists then add to the mat file
                        a = load(logData);
                        transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file
                        %save log
                        save(logData, 'transformsData');
                    else
                        % %save log
                        save(logData, 'transformsData');
                    end

                end

                %  give end effector pose to object and update object  parameters
                object.pose = endEffector;
                newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
                object.mesh.Vertices = newPoints(:, 1:3);
                drawnow()

            end

            % plot object at the final position
            object.pose = object.destiny;
            newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
            object.mesh.Vertices = newPoints(:, 1:3);
            drawnow()
        end

        %% Function for interpolation of UR3
        function qMatrixUR3 = interpolateJointAnglesUR3(q1, q2, steps)
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrixUR3 = nan(steps, 6); % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR3(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end
        end

        %%  Function for interpolation of UR5
        function qMatrixUR5 = interpolateJointAnglesLinearUR5(q1, q2, steps)
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrixUR5 = nan(steps, 7); % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR5(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end
        end

    end
end
%
