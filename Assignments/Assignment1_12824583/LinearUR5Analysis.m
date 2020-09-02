classdef LinearUR5Analysis
    %Class used to analyse details for the UR5.

    properties
        volume;
        transveralReach;
        verticalReach;
        arcRadius;
        computedVolume;
    end

    methods
        % Fucntion to ge the point cloud volume of the workspace.

        function pointCloud = UR5Volume(self, UR5, degrees)
            stepRads = deg2rad(degrees);
            qlim = UR5.model.qlim;

            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:6, 2)-qlim(1:6, 1)) / stepRads + 1));
            pointCloud = zeros(pointCloudeSize, 3);
            counter = 1;
            tic % start counter

            for q1 = qlim(1, 1):stepRads:qlim(1, 2)
                for q2 = qlim(2, 1):stepRads:qlim(2, 2)
                    for q3 = qlim(3, 1):stepRads:qlim(3, 2)
                        for q4 = qlim(4, 1):stepRads:qlim(4, 2)
                            for q5 = qlim(5, 1):stepRads:qlim(5, 2)
                                % q6 =0; % no need to worry about joint 6 Assume 0
                                for q6 = qlim(6, 1):stepRads:qlim(6, 2)
                                    q7 = 0;
                                    q = [q1, q2, q3, q4, q5, q6, q7];

                                    tr = UR5.model.fkine(q);
                                    pointCloud(counter, :) = tr(1:3, 4)'; % ' is to get the inverse
                                    counter = counter + 1;
                                    if mod(counter/pointCloudeSize*100, 1) == 0
                                        disp(['After ', num2str(toc), ' seconds, completed ', num2str(counter / pointCloudeSize * 100), '% of poses of Linear UR5'])
                                        self.volume = pointCloud;
                                    end
                                end
                            end
                        end
                    end
                end
            end

            % 2.6 Create a 3D model showing where the end effector can be over all these samples.
            view(3);
            plot3(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3), 'g.');

        end

        %% Function to get the trasversal Reach and radius of UR5
        function TransveralReach = getTransveralReach(self, pointclouds, UR5)

            maxX = max(pointclouds(:, 1)) - UR5.model.base(1, 4);
            minX = min(pointclouds(:, 1)) - UR5.model.base(1, 4);
            maxY = max(pointclouds(:, 2)) - UR5.model.base(2, 4);
            minY = min(pointclouds(:, 2)) - UR5.model.base(2, 4);
            maxZ = max(pointclouds(:, 3)) - UR5.model.base(3, 4);
            minZ = min(pointclouds(:, 3)) - UR5.model.base(3, 4);

            xAxisReach = max(maxX, abs(minX));
            yAxisReach = max(maxY, abs(minY));
            TransveralReach = max(xAxisReach, yAxisReach);
            self.transveralReach = TransveralReach;
        end

        %% Function to get the vertical  Reach and radius of UR5
        function verticalReach = getVerticalReach(self, pointclouds, UR5)

            maxX = max(pointclouds(:, 1)) - UR5.model.base(1, 4);
            minX = min(pointclouds(:, 1)) - UR5.model.base(1, 4);
            maxY = max(pointclouds(:, 2)) - UR5.model.base(2, 4);
            minY = min(pointclouds(:, 2)) - UR5.model.base(2, 4);
            maxZ = max(pointclouds(:, 3)) - UR5.model.base(3, 4);
            minZ = min(pointclouds(:, 3)) - UR5.model.base(3, 4);

            xAxisReach = max(maxX, abs(minX));
            yAxisReach = max(maxY, abs(minY));

            verticalReach = max(maxZ, abs(minZ));

        end

        %% Function to get the Arc radius
        function arcRadius = getArcRadius(self, pointclouds, UR5)

            maxX = max(pointclouds(:, 1)) - UR5.model.base(1, 4);
            minX = min(pointclouds(:, 1)) - UR5.model.base(1, 4);
            maxY = max(pointclouds(:, 2)) - UR5.model.base(2, 4);
            minY = min(pointclouds(:, 2)) - UR5.model.base(2, 4);
            maxZ = max(pointclouds(:, 3)) - UR5.model.base(3, 4);
            minZ = min(pointclouds(:, 3)) - UR5.model.base(3, 4);

            xAxisReach = max(maxX, abs(minX));
            yAxisReach = max(maxY, abs(minY));
            transveralReach = max(xAxisReach, yAxisReach);
            verticalReach = max(maxZ, abs(minZ));
            arcRadius = (verticalReach / 2) + ((transveralReach)^2 / 8 * verticalReach);
        end

        %% plot the volume of the arm

        function workspace = plotArmVolume(self, vol)
            [k, workspace] = convhull(vol(:, 1), vol(:, 2), vol(:, 3));
            trisurf(k, vol(:, 1), vol(:, 2), vol(:, 3), 'Facecolor', 'red');

        end

    end
end
