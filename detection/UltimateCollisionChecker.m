classdef UltimateCollisionChecker < handle & ParentChild
    properties
        pc_type = "UCC";
    end
%Attached Children are all meant to be Robot Q position FIFO queues
    methods
        function collision = check_collision(self, calling_q_matrix, calling_fifo)
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
                    overrun = longestFIFO - height(fifo_data)
                    if overrun > 0
                        fifo_data = padarray(fifo_data,[overrun 0],'replicate','post');
                    end
                end
                ultimate_q_array(:,i) = fifo_data;
            end
            %STAGE 2: GENERATE BOXES
            for i = 1:num_children
                %generate boxes for Q array on one robot
                %get robot controller
                controller = self.attached_child{i}.assigned_robot;
                for j = 1:longestFIFO
                    cubes = controller.build_detection_cubes(ultimate_q_array{j,i});
                    ultimate_dcube_array{j,i} = cubes;
                end
            end

            collision = false;
            %STAGE 3: THE LARGE CUBE COLLIDER
            % theProfiler = false;
            % if longestFIFO > 40
            %     profile on -timestamp -historysize 10000000 -timer performance
            %     theProfiler = true;
            % end
            for rowSelector = 1:height(ultimate_dcube_array)
                if ~collision
                    for i = 1:num_children
                        if ~collision
                            for j = i+1:num_children
                                if ~collision
                                    for CubeSelector = 1:length(ultimate_dcube_array{rowSelector,i})
                                        if ~collision
                                            for CubeSelectorTarget = 1:length(ultimate_dcube_array{rowSelector,i})
                                                if ~collision
                                                    collision_test = ultimate_dcube_array{rowSelector,i}{1,CubeSelector}.check_dc(ultimate_dcube_array{rowSelector,j}{1,CubeSelectorTarget});
                                                    if collision_test
                                                        collision = true;
                                                        self.drawCollisionData(rowSelector, i, CubeSelector, j, CubeSelectorTarget, ultimate_dcube_array);
                                                        error("Robots will collide.");       
                                                    end
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end

            % if theProfiler
            %     profile viewer
            % end
            
            return
        end

        function drawCollisionData(self, row, robot1, offendingcube, robot2, offendingcube2, dcube_array)
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