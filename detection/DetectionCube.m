classdef DetectionCube < GenericRenderable
    properties
        dco (1,1) DetectionController {}
        
        pc_type = "DetectionCube";
        detected_objects;

        cached_points;
        cached_hull;
    end
    methods
        function self = DetectionCube(ply_file, dco, transform)

            %call GenericRenderable constructor
            self@GenericRenderable(ply_file);
            self.dco = dco;
            self.set_transform_4by4(transform);
        end

        function objects_detected = tick(self)
            objects_detected = cell.empty;
            %hull = convhulln(self.cached_points);
            objects = self.dco.tracked_objects;
            objects_length = length(objects);
            for i = 1:objects_length
                hull2 = convhull(vertcat(self.cached_points, objects{i}.current_transform(1:3,4)'));
                if isequal(self.cached_hull, hull2)
                    objects_detected{end+1} = objects{i};
                end
            end    
            self.detected_objects = objects_detected;
        end

        function set_transform_4by4(self, matrix)
            self.current_transform = matrix;
            self.needsRedraw = true;
            

            %recompute points used for convex hull detection once for speed

            local_tri = self.tri;
            local_pts = self.pts;
            local_matrix = self.current_transform;
            % pts_height = height(local_pts);
            % 
            % new_pts = nan(pts_height, 3);
            % 
            % for j = 1:pts_height
            %     thePoint = local_pts(j,:).';
            %     thePoint(4,1) = 1;
            %     newPosition = local_matrix * thePoint;
            %     transposed = newPosition.';
            %     self.cached_points(j,:) = transposed(1:3);
            % end

            local_pts = local_pts';
            local_pts(4,:) = 1;
            local_pts = local_matrix * local_pts;
            self.cached_points = local_pts(1:3,:)';
            self.cached_hull = convhull(self.cached_points);
        end

        function render_optional(self)
            self.draw_handle.FaceColor = 'magenta';
            self.draw_handle.FaceAlpha = 0.3;

        end
    end
end