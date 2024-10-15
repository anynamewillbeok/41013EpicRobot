classdef Rubbish < GenericRenderable
    properties

    end

    methods
        function tick(self)
            %dont really have to do anything
            number_of_ticks = number_of_ticks + 1;
        end

        function render_optional(self)
            self.draw_handle.FaceColor = 'red';
        end
    end
end