classdef DetectionController < handle
    properties
        tracked_objects
    end

    methods
        function self = DetectionController()

        end
        function register(self, theThing)
            self.tracked_objects{end+1} = theThing;
        end
    end
end