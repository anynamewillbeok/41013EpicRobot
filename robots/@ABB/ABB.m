classdef ABB < RobotBaseClassNT
    %% ABB 20kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'ABB';
    end
    
    methods
%% Constructor
        function self = ABB(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			%self.model.base = self.model.base.T * baseTr;
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            %LinearUR5 section

            link(1) = Link('d',0.445,'a',0.1404,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0.25,'a',0.7,'alpha', 0, 'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',-0.25,'a',0.115,'alpha',pi/2,'offset',pi/2);
            link(4) = Link('d',0.8,'a',0,'alpha',pi/2);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'offset',pi);
            link(6) = Link('d',0.05);
            
            
            
            
            
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
