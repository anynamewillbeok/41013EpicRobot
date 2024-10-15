classdef (Abstract) Tickable < handle
    properties
        number_of_ticks = 0;
    end
    
    methods(Abstract)
        tick(self)
    end
end