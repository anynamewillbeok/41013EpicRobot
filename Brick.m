classdef Brick
    properties
        tri
        pts
    end
    
    methods
%% Constructor: Create brick instance using local file
        function self = Brick()
            % Read the Brick.ply file
            [self.tri, self.pts] = plyread('HalfSizedRedGreenBrick.ply', 'tri');
        end
%% Create series of modified ply files to make a brick trajectory
        function modified_pts_cell = make_position_matricies(self, desired_poses, animation_steps)

            modified_pts_cell = cell(1, animation_steps);

            for i = 1:animation_steps

                translation_vector = desired_poses(i).t;
                rotation_matrix = desired_poses(i).R;
                x = translation_vector(1);
                y = translation_vector(2);
                z = translation_vector(3);

                original_pts = self.pts;
                rotated_pts = zeros(size(original_pts));

                for j = 1:size(original_pts, 1)
                    rotated_pts(j, :) = (rotation_matrix * original_pts(j, :)')';
                end

                translated_and_rotated_pts = rotated_pts + [x, y, z];

                modified_pts_cell{i} = translated_and_rotated_pts;
            end
        end
%% Set a brick pose
function position = set_brick_pose(self, x, y, z, rotation_matrix)
    % Set the brick position considering translation and rotation
    translation_vector = [x, y, z];
    
    % Apply rotation to the original points
    rotated_pts = (rotation_matrix * self.pts')';
    
    % Translate the rotated points
    position = rotated_pts + repmat(translation_vector, size(rotated_pts, 1), 1);
        end
    end
end