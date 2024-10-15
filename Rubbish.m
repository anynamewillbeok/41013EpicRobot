classdef Rubbish < GenericRenderable
    properties

    end

    methods
        function tick(self)
            %dont really have to do anything
        end

        function render_optional(self)
            self.draw_handle.FaceColor = 'red';
        end
    end
end