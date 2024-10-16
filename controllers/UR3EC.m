classdef UR3EC < handle & ParentChild & Tickable

    methods
        function tick(self)
            number_of_ticks = number_of_ticks + 1;
        end
    end

end