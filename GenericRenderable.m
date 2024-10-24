classdef (Abstract) GenericRenderable < handle & ParentChild & Tickable & matlab.mixin.Copyable
    properties
        tri(:,3) double {}
        pts(:,3) double {}
        
    end
    properties(SetAccess = protected)
        needsRedraw(1,1) logical {} = true;
        current_transform (4,4) double {mustBeNonNan} = eye(4)
    end

    properties(SetAccess = public)
        needsRepatch(1,1) logical {} = false;
    end
    properties
        draw_handle (1,:) matlab.graphics.primitive.Patch {}  
    end

    methods
        %% Constructor: Create Renderable instance using local file
        function self = GenericRenderable(ply_file)
            % Read the ply file
            pc_type = "GenericRenderable";
            
            [self.tri, self.pts] = plyread(ply_file, 'tri');
            
            
        end
        %% Set transform for object
        function set_transform_4by4(self, matrix)

            self.current_transform = matrix;
            self.needsRedraw = true;

        end
        %% Render Object
        function return_object = render(self)

            %if ~isnan(self.draw_handle)
            
            %end

            

            if self.needsRedraw
                % try
                %     delete(self.draw_handle)
                % catch
                % end



                local_tri = self.tri;
                local_pts = self.pts;
                local_matrix = self.current_transform;
                
                %pts_height = height(local_pts);
                %new_pts = nan(pts_height, 3);
                % for j = 1:pts_height
                %     thePoint = local_pts(j,:).';
                %     thePoint(4,1) = 1;
                %     newPosition = local_matrix * thePoint;
                %     transposed = newPosition.';
                %     new_pts(j,:) = transposed(1:3);
                % end
                % Stupid, extremely terrible code that transforms ONE POINT
                % AT A TIME instead of making use of Matlab's supreme
                % matrix computational power

                %New code
                local_pts = local_pts';
                local_pts(4,:) = 1;
                local_pts = local_matrix * local_pts;
                new_pts = local_pts(1:3,:)';
                %End of new code

                hold on
                if isempty(self.draw_handle) | self.needsRepatch
                    self.draw_handle = patch('Faces',local_tri,'Vertices',new_pts, 'FaceColor', [0 0.1 0.3], 'EdgeColor', 'none');
                    self.needsRepatch = false;
                else
                    self.draw_handle.Vertices = new_pts;
                end
                       
                self.needsRedraw = false;     
                return_object = self.draw_handle;
                self.render_optional();
                hold off
            end

            if ~isempty(self.attached_child)
                for i = 1:length(self.attached_child)
                    self.attached_child{i}.render();
                end
            end
        end

        function render_optional(self)
        end
    end
end