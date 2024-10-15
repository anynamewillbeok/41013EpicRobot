classdef (Abstract) Tickable < handle
    methods(Abstract)
        tick(self)
    end
end