classdef RecontructObjectNonRGB < handle
    %Class made with the purpose of creating and handling object in the
    %enviroment. Source code provided from Week 4.

    properties
        vertexCount;
        pose;
        midPoint;
        verts;
        mesh;
        position;
        orientation;
        middlePoint
    end

    methods

        function obj = RecontructObjectNonRGB(file, modelOrientation, modelPosition)
            hold on;
            [f, v, data] = plyread(file, 'tri'); %#ok<ASGLU>
            obj.mesh = trisurf(f, v(:, 1), v(:, 2), v(:, 3) ...
                , 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            obj.vertexCount = size(v, 1);
            obj.midPoint = sum(v) / obj.vertexCount;
            obj.verts = v - repmat(obj.midPoint, obj.vertexCount, 1);

            obj.orientation = makehgtform('xrotate', modelOrientation(1, 1), 'yrotate', modelOrientation(1, 2), 'zrotate', modelOrientation(1, 3));
            obj.position = makehgtform('translate', [modelPosition(1, 1), modelPosition(1, 2), modelPosition(1, 3)]);
            obj.pose = eye(4) * obj.orientation * obj.position;
            updatedPoints = [obj.pose * [obj.verts, ones(obj.vertexCount, 1)]']';
            obj.mesh.Vertices = updatedPoints(:, 1:3);
            %drawnow();
        end

    end
end
