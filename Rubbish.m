classdef Rubbish < GenericRenderable
    properties
        pc_type = "Rubbish";
    end

    methods
        function tick(self)
            %dont really have to do anything
            self.number_of_ticks = self.number_of_ticks + 1;
        end

        function render_optional(self)
            self.draw_handle.FaceColor = 'red';
        end
    end
end