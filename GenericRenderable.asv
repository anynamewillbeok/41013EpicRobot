classdef GenericRenderable < handle
    properties
        tri
        pts
        current_transform = eye(4)
        attached_parent
        attached_child = createArray(0,0,'GenericRenderable')
        draw_handle
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

        end
        %% Render Object
        function render(self)
            
            %if ~isnan(self.draw_handle)
                try 
                    delete(self.draw_handle)
                catch
                end
            %end

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
            hold off

            if ~isempty(self.attached_child)
                for i = 1:length(self.attached_child)
                    self.attached_child(i).render()
                end
            end
        end
        %% Attach new parent
        function attach_parent(self, new_parent)
            if ~isempty(self.attached_parent)
                self.attached_parent.detach_child(self) 
            end

            self.attached_parent = new_parent;
            self.attached_parent.attach_child(self)
        end
        %% Detach
        function detach_child(self, object_to_detach)
            if ~isempty(self.attached_child)
                for i = 1:length(self.attached_child)
                    if self.attached_child(i) == object_to_detach
                        self.attached_child(i) = [];
                    end
                end
            end
        end
        %% Attach Child
        function attach_child(self, child)
           self.attached_child(end+1) = child;

        end
        %% Make Orphan
        function orphan(self)

            if ~isempty(self.attached_parent)
                self.attached_parent.detach_child(self)
            end
            self.attached_parent = [];
            
        end
    end
end