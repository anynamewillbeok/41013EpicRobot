classdef FIFO < handle
    properties(SetAccess = private)
        active(1,:)
        queue(:,:)
    end

    methods
        function self = FIFO(width)
            active = createArray(1,width,'double');
            queue = createArray(0,width,'double');
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

        function value = is_empty(self)
            value = isempty(self.queue);
        end
    end
end
