classdef (Abstract) Tickable < handle
    properties
        number_of_ticks(1,1) uint64 {mustBeInteger} = 0;
    end

    methods(Abstract)
        tick(self)
    end
end