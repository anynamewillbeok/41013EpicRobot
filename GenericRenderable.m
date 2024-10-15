classdef GenericRenderable < handle
    properties
        tri
        pts
    end
    
    methods
%% Constructor: Create Renderable instance using local file
function self = GenericRenderable(ply_file)
            % Read the ply file
            [self.tri, self.pts] = plyread(ply_file, 'tri');
        end
%% Set a pose and scale it
function position = set_brick_pose(self, x, y, z, rotation_matrix)
    
    % Scale the Renderable
    scale_factor = 0.25;
    scaled_pts = self.pts*scale_factor;
    
    % Set the fin position considering translation and rotation
    translation_vector = [x, y, z];
    
    % Apply rotation to the original points
    rotated_pts = (rotation_matrix * scaled_pts')';
    
    % Translate the rotated points
    position = rotated_pts + repmat(translation_vector, size(rotated_pts, 1), 1);
        end
    end
end