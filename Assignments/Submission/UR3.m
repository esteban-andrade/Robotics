classdef UR3 < handle
    properties
        %> Robot model
        model;
        volume = [];
        computedVolume;
        transveralReach;
        verticalReach;
        arcRadius;
        name;
        %> workspace
        workspace = [-2, 2, -2, 2, -0.3, 2];
        % workspace = [-1.6 1.6 -1.6 1.6 -0.3 1];  %change workspace for easier approach
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';
    end

    methods %% Class for UR3 robot simulation


        function self = UR3(acronym);
            self.name = ['UR 3 ', acronym];
            self.GetUR3Robot(self.name);
            self.PlotAndColourRobot(); %

            drawnow

        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self, name)
            pause(0.001);
            %name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %These were the default DH PARAMATERS provided from the manifacturer

            %             L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            %             L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',-pi/2); % was 'offset',-pi/2
            %             L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            %             L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', -pi/2); % was 'offset',-pi/2
            %             L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            %             L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            L1 = Link('d', 0.1519, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360, 360]), 'offset', 0);
            L2 = Link('d', 0, 'a', -0.24365, 'alpha', 0, 'qlim', deg2rad([-90, 90]), 'offset', -pi/2); % was 'offset',-pi/2
            L3 = Link('d', 0, 'a', -0.21325, 'alpha', 0, 'qlim', deg2rad([-360, 360]), 'offset', 0);
            L4 = Link('d', 0.11235, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360, 360]), 'offset', -pi/2); % was 'offset',-pi/2
            L5 = Link('d', 0.08535, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-360, 360]), 'offset', 0);
            L6 = Link('d', 0.0819, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360, 360]), 'offset', 0);

            self.model = SerialLink([L1, L2, L3, L4, L5, L6], 'name', name);
            self.name = name;
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self) %robot,workspace)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['UR3Link', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [faceData, vertexData, plyData{self.model.n + 1}] = plyread(self.toolModelFilename, 'tri');
                self.model.faces{self.model.n+1} = faceData;
                self.model.points{self.model.n+1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);
            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles, 'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red, ...
                        plyData{linkIndex + 1}.vertex.green, ...
                        plyData{linkIndex + 1}.vertex.blue] / 255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        %% draw the volume of the arm Using point clouds

        function DrawVolumeArm(self, degrees)
            stepRads = deg2rad(degrees);
            qlim = self.model.qlim;

            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5, 2)-qlim(1:5, 1)) / stepRads + 1));
            pointCloud = zeros(pointCloudeSize, 3);
            counter = 1;
            tic % start counter

            for q1 = qlim(1, 1):stepRads:qlim(1, 2)
                for q2 = qlim(2, 1):stepRads:qlim(2, 2)
                    for q3 = qlim(3, 1):stepRads:qlim(3, 2)
                        for q4 = qlim(4, 1):stepRads:qlim(4, 2)
                            for q5 = qlim(5, 1):stepRads:qlim(5, 2)
                                q6 = 0; % no need to worry about joint 6 Assume 0
                                %for q6 = qlim(6,1):stepRads:qlim(6,2)

                                q = [q1, q2, q3, q4, q5, q6];

                                tr = self.model.fkine(q);
                                pointCloud(counter, :) = tr(1:3, 4)'; % ' is to get the inverse

                                counter = counter + 1;
                                if mod(counter/pointCloudeSize*100, 1) == 0
                                    disp(['After ', num2str(toc), ' seconds, completed ', num2str(counter / pointCloudeSize * 100), '% of poses of UR3'])
                                    self.volume = pointCloud; %store point cloud volume
                                end
                                %end
                            end
                        end
                    end
                end
            end

            plot3(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3), 'r.'); % plot UR point cloud volume

        end

        %%    get Reach of UR3
        function getReach(self)

            maxX = max(self.volume(:, 1)) - self.model.base(1, 4);
            minX = min(self.volume(:, 1)) - self.model.base(1, 4);
            maxY = max(self.volume(:, 2)) - self.model.base(2, 4);
            minY = min(self.volume(:, 2)) - self.model.base(2, 4);
            maxZ = max(self.volume(:, 3)) - self.model.base(3, 4);
            minZ = min(self.volume(:, 3)) - self.model.base(3, 4);

            xAxisReach = max(maxX, abs(minX));
            yAxisReach = max(maxY, abs(minY));
            self.transveralReach = max(xAxisReach, yAxisReach); %Calculate Transversal arm reach
            self.verticalReach = max(maxZ, abs(minZ)); %Calculate Vertical arm reach
            self.arcRadius = (self.verticalReach / 2) + ((self.transveralReach)^2 / 8 * self.verticalReach); % calculate ARC radius from arm
        end

        %% point cloud  of volumetric arm

        function plotArmVolume(self)
            [k, self.computedVolume] = convhull(self.volume(:, 1), self.volume(:, 2), self.volume(:, 3));
            trisurf(k, self.volume(:, 1), self.volume(:, 2), self.volume(:, 3), 'Facecolor', 'cyan');

        end
    end
end