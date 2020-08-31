classdef RecontructObject < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        vertexCount;
        pose;
        midPoint;
        verts;
        destiny; %1 for UR3, 2 for UR5, 0 non assigned
        
        mesh;
        position;
        orientation;
        
        middlePoint
    end
    
    methods
        
        
        function obj = RecontructObject(file,modelOrientation,modelPosition,destinyPosition)
             hold on;
            [f,v,data] = plyread(file,'tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            
            
            obj.mesh =  trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            obj.vertexCount = size(v,1);
            obj.midPoint = sum(v)/obj.vertexCount;
            obj.verts= v - repmat(obj.midPoint,obj.vertexCount,1);
            
            obj.orientation =  makehgtform('xrotate',modelOrientation(1,1),'yrotate',modelOrientation(1,2),'zrotate',modelOrientation(1,3));
            obj.position=makehgtform('translate',[modelPosition(1,1),modelPosition(1,2),modelPosition(1,3)]);
            obj.pose = eye(4)*obj.orientation*obj.position;
            updatedPoints = [obj.pose * [obj.verts,ones(obj.vertexCount,1)]']';
            obj.mesh.Vertices =  updatedPoints(:,1:3);
            obj.destiny=destinyPosition;
            %drawnow();
        end
        
    end
end

