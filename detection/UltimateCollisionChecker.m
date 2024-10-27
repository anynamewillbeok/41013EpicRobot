classdef UltimateCollisionChecker < handle & ParentChild
    properties
        pc_type = "UCC";
    end
%Attached Children are all meant to be Robot Q position FIFO queues
    methods
        function collision = check_collision(self, calling_q_matrix, calling_fifo)

            tic;

            % theProfiler = false;
            % if height(calling_q_matrix) > 40
            %     profile on -timestamp -historysize 10000000 -timer performance
            %     theProfiler = true;
            % end
            % 
            %figure out which child called the function
            num_children = length(self.attached_child);
            calling_fifo_index = 0;
            for i = 1:num_children
                if self.attached_child{i} == calling_fifo
                    calling_fifo_index = i;
                    break
                end
            end
            if calling_fifo_index == 0
                error("UltimateCollisionChecker: Could not find calling FIFO!");
            end
            %construct terrifyingly ginormous Q array, padding if required
            %first figure out which FIFO is longest

            fifo_lengths = createArray(1,num_children,"double");
            
            for i = 1:num_children
                fifo_lengths(i) = self.attached_child{1,i}.get_active_and_queue_length();
                if i == calling_fifo_index
                    fifo_lengths(i) = fifo_lengths(i) + height(calling_q_matrix);
                end
            end
            longestFIFO = max(fifo_lengths);

            

            ultimate_q_array = cell(longestFIFO, num_children);
            ultimate_dcube_array = cell(longestFIFO, num_children);
             %row 1 corresponds to the "active" region, also why q
                %array is one higher than longestFIFO

            if longestFIFO > 40

            end
            



            %STAGE 1: IMPORT DATA
            for i = 1:num_children
                if ~(calling_fifo_index == i)
                    fifo_data = self.attached_child{i}.get_active_and_queue(longestFIFO);
                    fifo_data = num2cell(fifo_data,2); %turn into vertical cell array, each entry contains the entire Q state for one tick
                else %if we are the caller... build fifo data that would correspond to what we want
                    data_length = self.attached_child{i}.get_active_and_queue_length();
                    fifo_data = cell(height(calling_q_matrix),1);
                    data = self.attached_child{i}.get_active_and_queue(data_length);
                    fifo_data(1:data_length,1) = num2cell(data,2);
                    fifo_data(data_length + 1:data_length + height(calling_q_matrix),1) = num2cell(calling_q_matrix,2);
                    %Pad if data too short
                    overrun = longestFIFO - height(fifo_data);
                    if overrun > 0
                        fifo_data = padarray(fifo_data,[overrun 0],'replicate','post');
                    end
                end
                ultimate_q_array(:,i) = fifo_data;
            end
            %STAGE 2: GENERATE BOXES
            %children = self.attached_child;
            % 
            %STAGE 2.-1: BUILD PARALLELISATION DATA
            % children = self.attached_child;
            % linkdata = cell(1,num_children);
            % robot_base_transform = cell(1,num_children);
            % robot = cell(1,num_children);
            % robot_p_dc_t = cell(1,num_children);
            % for i = 1:num_children
            %     linkdata{i} = children{i}.assigned_robot.linkdata;
            %     robot_base_transform{i} = children{i}.assigned_robot.base_transform;
            %     robot{i} = children{i}.assigned_robot;
            %     robot_p_dc_t{i} = children{i}.assigned_robot.p_dc_t;
            % end

            % addAttachedFiles(gcp,["Link.m" "transl.m" "trscale.m"]);

            %ticBytes;
            
            for i = 1:num_children
            %for i = 1:num_children %tried to use parfor here, unfortunately has massive overhead so we just use normal for
                %generate boxes for Q array on one robot
                for j = 1:longestFIFO
                    ultimate_dcube_array{j,i} = self.attached_child{i}.assigned_robot.build_detection_cubes(ultimate_q_array{j,i});
                    %ultimate_dcube_array{j,i} = build_detection_cubes_static(ultimate_q_array{j,i}, linkdata{i}, robot_base_transform{i}, robot_p_dc_t{i}) %Use if you want to try parallelisation
                    %ultimate_dcube_array{j,i} = cubes;
                end
            end

            %tocBytes;

            
            %STAGE 3: THE LARGE CUBE COLLIDER

            theheight2 = height(ultimate_dcube_array);

            %collisiontest = createArray(theheight2,1,"logical");
            thecollision = false;
            collision = false;
            
            for rowSelector = 1:theheight2
                sliced_ultimate_dcube_array = ultimate_dcube_array(rowSelector,:);
                thecollision = false;
                if ~thecollision
                    for i = 1:num_children
                        %if ~thecollision
                        for j = i+1:num_children
                            %if ~thecollision
                            for CubeSelector = 1:length(sliced_ultimate_dcube_array{i})
                                %if ~thecollision
                                for CubeSelectorTarget = 1:length(sliced_ultimate_dcube_array{j})
                                    %if ~thecollision
                                    collision_test = sliced_ultimate_dcube_array{i}{CubeSelector}.check_dc(sliced_ultimate_dcube_array{j}{CubeSelectorTarget});
                                    if collision_test
                                        thecollision = true;
                                        %collisiontest(rowSelector) = true;
                                        collision = true;
                                        self.drawCollisionData(rowSelector, i, CubeSelector, j, CubeSelectorTarget, ultimate_dcube_array);
                                        toc;
                                        warn("Robots will collide.");
                                    end

                                end

                            end

                        end

                    end
                end
            end
            %collision = any(collisiontest);

            % if theProfiler
            %      profile viewer
            % end

            toc;
            
            return
        end

        function drawCollisionData(~, row, robot1, offendingcube, robot2, offendingcube2, dcube_array)
            robot_1_cubes = dcube_array{row, robot1};
            robot_2_cubes = dcube_array{row, robot2};

            for i = 1:length(robot_1_cubes)
                handle = robot_1_cubes{i}.render();
                if i == offendingcube
                handle.FaceColor = "red";
                handle.FaceAlpha = 0.7;
                else
                handle.FaceColor = [0.4660 0.6740 0.1880];
                handle.FaceAlpha = 0.4;
                end
            end
            for i = 1:length(robot_2_cubes)
                handle = robot_2_cubes{i}.render();
                if i == offendingcube2
                handle.FaceColor = "red";
                handle.FaceAlpha = 0.7;
                else
                handle.FaceColor = [0.4660 0.6740 0.1880];
                handle.FaceAlpha = 0.4;
                end
            end
        end
            

    end
end