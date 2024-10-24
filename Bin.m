classdef Bin < GenericRenderable
    properties
        colour(1,3) double {mustBeNonNan mustBeNonnegative}
        pc_type = "Bin"
        dco(1,1) DetectionController
        dcube(1,1)
        speed = 0.008;
    end

    methods
        function self = Bin(ply_file, dco, transform)
            self@GenericRenderable(ply_file);
            self.dco = dco;
            %create Detection Cube just above bin      
            self.dcube = DetectionCube('Cube.ply',dco,transform * transl(0,0,0.61) * trscale(0.95,0.95,1.2));
            self.current_transform = transform;
            self.dcube.attach_parent(self);
        end

        function tick(self)
            self.number_of_ticks = self.number_of_ticks + 1;
            %query for objects inside the cube and capture the orphans
            objects = self.dcube.tick();
            if ~isempty(objects)
                for i = 1:length(objects)
                    if objects{i}.is_orphan()
                        objects{i}.attach_parent(self)
                    end
                end
            end
            %copy code for conveyor belt

            %move everything EXCEPT detection cubes


            
            num_children = length(self.attached_child);
            c = self.current_transform;
            %translate each object by HOW MUCH
            %move objects in different direction according to rotation of
            %conveyor belt, but not translation or scale
            scale_x = sqrt(c(1,1)^2+c(1,2)^2+c(1,3)^2);
            scale_y = sqrt(c(2,1)^2+c(2,2)^2+c(2,3)^2);
            scale_z = sqrt(c(3,1)^2+c(3,2)^2+c(3,3)^2);

            c(1:3,1) = c(1:3,1) / scale_x;
            c(1:3,2) = c(1:3,2) / scale_y;
            c(1:3,3) = c(1:3,3) / scale_z;

            %move along wherever the negative Z axis points
            movement = c(1:3,3)';
            movement = movement * self.speed;
            applied_transform = transl(movement(1),movement(2),movement(3));
            
            for i = 1:num_children
                child_pos = self.attached_child{i}.current_transform;
                child_pos = child_pos * applied_transform;
                %only move objects if they are ABOVE the bin
                relative_position = child_pos * self.current_transform;
                if relative_position(3,4) > 0 && ~(self.attached_child{i}.pc_type == "DetectionCube")
                    self.attached_child{i}.set_transform_4by4(child_pos);
                end
                self.attached_child{i}.tick(); %also tick children
            end
        end
    end
end


