classdef CubeCache < handle & ParentChild
    properties(SetAccess = private)
        width
        
        active(1,:)
        queue(:,:)
        assigned_robot(1,1)
    end

    properties
        pc_type = "CubeCache";
    end

    methods
        function self = CubeCache(width)
            self.width = width;
            self.active = createArray(1,width,'cell');
            self.queue = createArray(0,width,'cell');
        end

        function vector = pull(self)
            vector = self.active;
            if ~isempty(self.queue)
            self.active = self.queue(1,:);
            self.queue(1,:) = [];
            end
        end

        function add(self,matrix)
            self.queue = vertcat(self.queue,matrix);
            if isempty(self.active)
                self.pull()
            end
        end

        function replace_queue(self,matrix)
            self.active = matrix(1,:);
            self.queue = matrix(2:height(matrix),:);
        end

        function length = get_active_and_queue_length(self)
            length = 1 + height(self.queue); %active + queue
            return
        end

        function array = get_active_and_queue(self, length)
            local_queue = createArray(length,self.width,'cell');
            local_queue(1,:) = self.active;
            if ~isempty(self.queue)
                local_queue(2:height(self.queue) + 1,:) = self.queue;
            end
            
            %replace empty entries with "false"
            array = local_queue;
            
        end

        function value = is_empty(self)
            value = isempty(self.queue) || isscalar(self.queue); %make code emit new paths if queue is one or we actually mistime trajectory generation by one tick causing stutter
        end
    end
end
