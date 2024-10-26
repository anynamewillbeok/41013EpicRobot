classdef FIFO < handle & ParentChild
    properties(SetAccess = private)
        width
        
        active(1,:)
        queue(:,:)
        assigned_robot(1,1)
    end

    properties
        pc_type = "FIFO"
    end

    methods
        function self = FIFO(width, creator)
            self.width = width;
            self.active = createArray(1,width,'double');
            self.queue = createArray(0,width,'double');
            self.assigned_robot = creator;
        end

        function vector = pull(self)
            vector = self.active;
            if ~isempty(self.queue)
            self.active = self.queue(1,:);
            self.queue(1,:) = [];
            end
        end

        function force_add(self,matrix)
            self.queue = vertcat(self.queue,matrix);
            if isempty(self.active)
                self.pull()
            end
        end

        function success = add(self, matrix)
            arguments
                self
                matrix(:,:) double {mustBeNonNan}
            end
            %call parent (Grand Collision Checker) for okay
            if isempty(self.attached_parent{1})
                error("FIFO has no Ultimate Collision Checker™ attached!");
                success = false;
                return
            else
                collision = self.attached_parent{1}.check_collision(matrix, self);
                if ~collision
                    self.force_add(matrix);
                end
                success = ~collision;
                return
            end
        end

        function length = get_active_and_queue_length(self)
            length = 1 + height(self.queue); %active + queue
            return
        end

        function array = get_active_and_queue(self, length)
            local_queue = createArray(1 + height(self.queue),self.width,'double');
            local_queue(1,:) = self.active;
            if ~isempty(self.queue)
                local_queue(2:height(self.queue) + 1,:) = self.queue;
            end
            
            %check if padding required
            overrun = length - height(local_queue); %length we have to pad by if requested length longer than whats going to be returned
            if overrun > 0
                array = padarray(local_queue,[overrun 0],"replicate","post");
            else
                array = local_queue;
            end
        end

        function value = is_empty(self)
            value = isempty(self.queue) || isscalar(self.queue); %make code emit new paths if queue is one or we actually mistime trajectory generation by one tick causing stutter
        end
    end
end
