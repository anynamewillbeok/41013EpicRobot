classdef A2UR3E < RobotBaseClassNT
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'A2UR3E';
    end
    
    methods
%% Constructor
        function self = A2UR3E(baseTr,useTool,toolFilename)
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
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.tool = self.toolTr;
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            %LinearUR5 section

            % link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            % link(2) = Link([0      0.1599  0       pi/2    0]);
            % link(1).qlim = [0.8 0.0];
            % link(2).qlim = [-360 360]*pi/180;

            %modded LinearUR5 section because 
            link(1) = Link('theta',0,'alpha',-pi/2,'prismatic','qlim',[-0.8, -0.06]);
            link(2) = Link('d',0.1599,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(1).d = 0; %i need to do this otherwise something explodes in RobotBaseClass

            %UR3e section

            %link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            %This link is unneeded as the base is replaced with the Linear
            %UR5 base
            
            link(3) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
