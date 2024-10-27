classdef DetectionCube < GenericRenderable
    properties
        dco (1,1) DetectionController {}

        transform_inverse (4,4) double {}
        
        pc_type = "DetectionCube";
        detected_objects;

        objects_detected_count = 1;

        box_limits(2,3) double

        cached_points;
    end
    methods
        function self = DetectionCube(dco, transform)

            %call GenericRenderable constructor
            self@GenericRenderable("0");
            self.dco = dco;
            self.set_transform_4by4(transform);
        end

        function objects_detected = tick(self)
            objects_detected = cell(1,self.objects_detected_count); 
            objects_detected_ticker = 1;
            %hull = convhulln(self.cached_points);
            objects = self.dco.tracked_objects;
            objects_length = length(objects);
            for i = 1:objects_length
                %hull2 = convhull(vertcat(self.cached_points, objects{i}.current_transform(1:3,4)'));
                point_pos = objects{i}.current_transform; %after all the optimisations this line here is actually the most expensive part of the whole tick
                if point_pos(1,4) >= self.box_limits(1,1) && point_pos(1,4) <= self.box_limits(2,1) && point_pos(2,4) >= self.box_limits(1,2) && point_pos(2,4) <= self.box_limits(2,2) && point_pos(3,4) >= self.box_limits(1,3) && point_pos(3,4) <= self.box_limits(2,3) 
                %if isequal(self.cached_hull, hull2)
                    %If an object is inside the GREEN bounding box: perform
                    %inverse transformatigon, check if between -0.5 and 0.5
                    inverse_point_pos = self.transform_inverse * point_pos;
                    if all(inverse_point_pos(1:3,4) >= -0.5 & inverse_point_pos(1:3,4) <= 0.5)
                        objects_detected{objects_detected_ticker} = objects{i};
                        objects_detected_ticker = objects_detected_ticker + 1;
                    end
                end
            end    
            if self.objects_detected_count ~= objects_detected_ticker - 1
                objects_detected(cellfun(@numel,objects_detected)==0) = [];
                self.objects_detected_count = objects_detected_ticker - 1;
            end
            self.detected_objects = objects_detected;
        end

        function intersects = check_dc(self, other_cube)
            arguments
                self
                other_cube DetectionCube
            end
            intersects = false;

            for i = 1:8 %%potential optimization: somehow check all points at the same time
                %point_pos = other_cube.cached_points(i,:);
                if other_cube.cached_points(i,1) >= self.box_limits(1,1) && other_cube.cached_points(i,1) <= self.box_limits(2,1) && other_cube.cached_points(i,2) >= self.box_limits(1,2) && other_cube.cached_points(i,2) <= self.box_limits(2,2) && other_cube.cached_points(i,3) >= self.box_limits(1,3) && other_cube.cached_points(i,3) <= self.box_limits(2,3) 
                    inverse_point_pos = self.transform_inverse * transl(other_cube.cached_points(i,1),other_cube.cached_points(i,2),other_cube.cached_points(i,3));
                    if all(inverse_point_pos(1:3,4) >= -0.5 & inverse_point_pos(1:3,4) <= 0.5)
                        intersects = true;
                        break
                    end
                end
            end
        end

        function set_transform_4by4(self, matrix)
            self.current_transform = matrix;
            self.needsRedraw = true;

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

            %Also calculate inverse of matrix 

            self.transform_inverse = inv(local_matrix);
            
            %This is the "green" bounding box of the "magenta" detection cube. 
            %Not the actual detection box itself.
            [self.box_limits(1,1),self.box_limits(2,1)] = bounds(local_pts(1,:)); %min x and max x
            %self.box_limits(2,1) = max(local_pts(1,:)); %max x
            [self.box_limits(1,2),self.box_limits(2,2)] = bounds(local_pts(2,:));
            %self.box_limits(1,2) = min(local_pts(2,:)); %min y
            %self.box_limits(2,2) = max(local_pts(2,:)); %max y
            [self.box_limits(1,3),self.box_limits(2,3)] = bounds(local_pts(3,:));
            %self.box_limits(1,3) = min(local_pts(3,:)); %min z
            %self.box_limits(2,3) = max(local_pts(3,:)); %max z

            %disp(self.box_limits);


        end

        function render_optional(self)
            self.draw_handle.FaceColor = 'magenta';
            self.draw_handle.FaceAlpha = 0.3;
        end
    end
end