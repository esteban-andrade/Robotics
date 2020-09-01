function [pointCloud] = UR5Volume(UR5, degrees)
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
                        end
                    end
                end
            end
        end
    end
end

% 2.6 Create a 3D model showing where the end effector can be over all these samples.
view(3);
plot3(pointCloud(:, 1), pointCloud(:, 2), pointCloud(:, 3), 'b.');

end
