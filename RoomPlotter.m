classdef RoomPlotter
    properties
        imgFloor 
        imgZY    
        imgZX    
    end
    
    methods
        function obj = RoomPlotter(floorImage, zyImage, zxImage)
            obj.imgFloor = imread(floorImage);
            obj.imgZY = imread(zyImage);
            obj.imgZX = imread(zxImage);
            obj.imgZY = rot90(obj.imgZY, 3);  
            obj.imgZX = rot90(obj.imgZX, 2);  
        end

        function plotRoom(obj)
            figure;
            hold on;
            axis equal;
            %% change for bigger room
            xlim([-10 0]);
            ylim([0 10]);
            zlim([0 10]);

            [X, Y] = meshgrid(-10:0, 0:10);
            Z = zeros(size(X));
            surface(X, Y, Z, 'CData', obj.imgFloor, 'FaceColor', 'texturemap');
            
            [Z2, Y2] = meshgrid(0:10, 0:10);
            X2 = zeros(size(Z2));
            surface(X2, Y2, Z2, 'CData', obj.imgZY, 'FaceColor', 'texturemap');
            
            [X3, Z3] = meshgrid(-10:0, 0:10); 
            Y3 = zeros(size(X3)); 
            surface(X3, Y3, Z3, 'CData', obj.imgZX, 'FaceColor', 'texturemap');
            
            xlabel('X-axis');
            ylabel('Y-axis');
            zlabel('Z-axis');
            view(3);
            rotate3d on; 
        end
    end
end
