classdef (Abstract) ParentChild < handle
    properties
        attached_parent (1,1) cell {} = cell(1,1);
        attached_child (1,:) cell {} = cell(1,0);
    end

    methods
        %% Attach new parent
        function attach_parent(self, new_parent)
            if ~isempty(self.attached_parent{1})
                self.attached_parent{1}.detach_child(self) 
            end

            self.attached_parent{1} = new_parent;
            self.attached_parent{1}.attach_child(self)
        end
        %% Detach
        function detach_child(self, object_to_detach)
            if ~isempty(self.attached_child)
                i = 1;
                while all(i >= 1 & i <= length(self.attached_child))
                    if self.attached_child{i} == object_to_detach
                        self.attached_child(i) = []; 
                        i = i - 1; 
                    end
                    i = i + 1;
                end
            end
        end
        %% Attach Child
        function attach_child(self, child)
           self.attached_child{end+1} = child; %curly braces here needed for cell array

        end
        %% Make Orphan
        function orphan(self)

            if ~isempty(self.attached_parent{1})
                self.attached_parent{1}.detach_child(self)
            end
            self.attached_parent{1} = [];
            
        end
    end
end