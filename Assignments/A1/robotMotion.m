classdef robotMotion
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        endEffector;
    end

    methods (Static)

        function motion(qMatrix, robot)

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
                        %if the file exists then add to the mat file
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

        %%

        function doubleMotion(qMatrixA, robot1, qMatrixB, robot2)


            logData1 = ['transforms-data-1.mat'];
            logData2 = ['transforms-data-2.mat'];

            qMatrix = [qMatrixA, qMatrixB];

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
                        %if the file exists then add to the mat file
                        a = load(logData1);

                        transformsData1 = [a.transformsData1; transformsData1]; %adds data to the end of the mat file

                        save(logData1, 'transformsData1');
                    else
                        %if the file does not exist, save
                        save(logData1, 'transformsData1');
                    end


                    if isfile(logData2)
                        %if the file exists then add to the mat file
                        b = load(logData2);

                        transformsData2 = [b.transformsData2; transformsData2]; %adds data to the end of the mat file

                        save(logData2, 'transformsData2');
                    else
                        %if the file does not exist, save
                        save(logData2, 'transformsData2');
                    end

                end


                drawnow()

            end
        end

        %%

        function objectMotion(qMatrix, robot, object, objectDestiny)
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
                        %if the file exists then add to the mat file
                        a = load(logData);

                        transformsData = [a.transformsData; transformsData]; %adds data to the end of the mat file

                        save(logData, 'transformsData');
                    else
                        %if the file does not exist, save
                        save(logData, 'transformsData');
                    end

                end
                object.pose = endEffector;
                newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
                object.mesh.Vertices = newPoints(:, 1:3);


                drawnow()

            end
            object.pose = objectDestiny;
            newPoints = [object.pose * [object.verts, ones(object.vertexCount, 1)]']';
            object.mesh.Vertices = newPoints(:, 1:3);


            drawnow()

        end

        %%
        function doubleObjectMotion(qMatrixA, robot1, object1, object1Destiny, qMatrixB, robot2, object2, object2Destiny)
            logData1 = ['transforms-data-1.mat'];
            logData2 = ['transforms-data-2.mat'];

            qMatrix = [qMatrixA, qMatrixB];

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
                        %if the file exists then add to the mat file
                        a = load(logData1);

                        transformsData1 = [a.transformsData1; transformsData1]; %adds data to the end of the mat file

                        save(logData1, 'transformsData1');
                    else
                        %if the file does not exist, save
                        save(logData1, 'transformsData1');
                    end


                    if isfile(logData2)
                        %if the file exists then add to the mat file
                        b = load(logData2);

                        transformsData2 = [b.transformsData2; transformsData2]; %adds data to the end of the mat file

                        save(logData2, 'transformsData2');
                    else
                        %if the file does not exist, save
                        save(logData2, 'transformsData2');
                    end

                end

                object1.pose = endEffector1;
                newPoints1 = [object1.pose * [object1.verts, ones(object1.vertexCount, 1)]']'; %#ok<NBRAK>
                object1.mesh.Vertices = newPoints1(:, 1:3);

                object2.pose = endEffector2;
                newPoints2 = [object2.pose * [object2.verts, ones(object2.vertexCount, 1)]']'; %#ok<NBRAK>
                object2.mesh.Vertices = newPoints2(:, 1:3);


                drawnow()

            end

            object1.pose = object1Destiny;
            newPoints1 = [object1.pose * [object1.verts, ones(object1.vertexCount, 1)]']'; %#ok<NBRAK>
            object1.mesh.Vertices = newPoints1(:, 1:3);

            object2.pose = object2Destiny;
            newPoints2 = [object2.pose * [object2.verts, ones(object2.vertexCount, 1)]']'; %#ok<NBRAK>
            object2.mesh.Vertices = newPoints2(:, 1:3);


            drawnow()


        end

        %%
        function qMatrixUR3 = interpolateJointAnglesUR3(q1, q2, steps)
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrixUR3 = nan(steps, 6); % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR3(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end
        end

        %%
        function qMatrixUR5 = interpolateJointAnglesLinearUR5(q1, q2, steps)
            s = lspb(0, 1, steps); % First, create the scalar function
            qMatrixUR5 = nan(steps, 7); % Create memory allocation for variables
            for i = 1:steps
                qMatrixUR5(i, :) = (1 - s(i)) * q1 + s(i) * q2; % Generate interpolated joint angles
            end

            %qMatrixUR5 = jtraj(q1,q2,steps);
        end


    end
end
