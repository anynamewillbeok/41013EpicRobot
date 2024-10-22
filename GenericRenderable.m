classdef (Abstract) GenericRenderable < handle & ParentChild & Tickable
    properties(SetAccess = immutable)
        tri(:,3) double {}
        pts(:,3) double {}
    end
    properties(SetAccess = protected)
        needsRedraw(1,1) logical {} = true;
    end
    properties
        current_transform (4,4) double {mustBeNonNan} = eye(4)
        draw_handle (1,:) matlab.graphics.primitive.Patch {}
        
    end

    methods
        %% Constructor: Create Renderable instance using local file
        function self = GenericRenderable(ply_file)
            % Read the ply file
            
            [self.tri, self.pts] = plyread(ply_file, 'tri');
            
            
        end
        %% Set a pose and scale it ONLY FOR TROUBLESHOOTING
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
        %% Set transform for object
        function set_transform_4by4(self, matrix)

            self.current_transform = matrix;
            self.needsRedraw = true;

        end
        %% Render Object
        function render(self)

            %if ~isnan(self.draw_handle)
            
            %end

            

            if self.needsRedraw
                try
                    delete(self.draw_handle)
                catch
                end

                local_tri = self.tri;
                local_pts = self.pts;
                local_matrix = self.current_transform;
                pts_height = height(local_pts);

                new_pts = nan(pts_height, 3);

                for j = 1:pts_height
                    thePoint = local_pts(j,:).';
                    thePoint(4,1) = 1;
                    newPosition = local_matrix * thePoint;
                    transposed = newPosition.';
                    new_pts(j,:) = transposed(1:3);
                end

                hold on
                self.draw_handle = patch('Faces',local_tri,'Vertices',new_pts, 'FaceColor', [0 0.1 0.3], 'EdgeColor', 'none');
                self.render_optional();
                self.needsRedraw = false;
                hold off
            end

            if ~isempty(self.attached_child)
                for i = 1:length(self.attached_child)
                    self.attached_child{i}.render()
                end
            end
        end

        function render_optional(self)
        end
    end
end