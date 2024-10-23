classdef UR3EC < handle & ParentChild & Tickable
    properties
        robot 
        present_queue_robot FIFO %Present queue holds
        present_queue_claw FIFO
        pc_type = "Robot"
        detection(1,1) DetectionController
        detection_cubes(1,:) DetectionCube

        path_lengths = [50,35,50,50,50,50,50,50];
    end

    properties(SetAccess = private)
        state(1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(state, 100000)}
        tracked_object
        tracked_object_info;
        tracked_object_velocity;
        current_q;
    end

    methods
        function self = UR3EC(transform, detection)
            self.robot = UR3eNT(transform);
            self.detection = detection;
            self.robot.model.animate(deg2rad([90,-120,-60,-90,90,0]));
            self.current_q = deg2rad([90,-120,-60,-90,90,0])
            
            self.present_queue_robot = FIFO(length(self.robot.model.links));
            self.present_queue_robot.add(deg2rad([90,-120,-60,-90,90,0]));

            %starting Q positions
            %Q = [0,0,0]
            %self.robot.model.animate
            self.state = 0;

            %setup detection cube
            %transform 1: towards the conveyor belt idk where that is
            %really 
            cube1_transform = transform
            cube1_transform = cube1_transform * transl(-0.3, 0.4, 0);
            cube1_transform = cube1_transform * trscale(1, 0.6, 0.5);
            cube1 = DetectionCube("Cube.ply",detection,cube1_transform);
            self.detection_cubes(1) = cube1;
        end


        function tick(self)
            self.number_of_ticks = self.number_of_ticks + 1;

            switch self.state
                case 0
                    if self.present_queue_robot.is_empty
                        detected_objects = self.detection_cubes(1).tick();
                        if ~isempty(detected_objects)
                            disp("Detected object!");
                            self.tracked_object = detected_objects(1); %select first object
                            self.tracked_object_info{1} = self.tracked_object{1}.current_transform;
                            self.state = 1000;
                        end
                    end
                    

                    %STAGE 0: Detect rubbish. 
                    %Once detected, emit pathway to being above rubbish's projected
                    %position with ending velocity. Increment state.
                    %Tracked object is set to the detected object.
                case 1
                    %STAGE 1: Dropdown
                    %Once present queue is empty, maintaining rubbish sideways velocity, emit path move
                    %end effector downwards towards rubbish, while opening grabber.
                    %Increment state.

                    %Perhaps use RMRC to generate path? has to be
                    %constantly downwards and movement slightly in a
                    %straight path
                    %in english: generate jacobian of current Q, calculate
                    %joint velocity vector by multiplying inverse of
                    %jacobian by desired velocity vector
                    %do this over and over again until we reach the
                    %rubbish? then emit

                    if self.present_queue_robot.is_empty

                    PATH_LENGTH = self.path_lengths(2);
                    %Generate path                  
                    %project
                    projected_distance = self.tracked_object_velocity * PATH_LENGTH;
                    projected_position = self.tracked_object{1}.current_transform * transl(projected_distance);
                    %render a rubbish here
                    projectedObject = copy(self.tracked_object{1});
                    projectedObject.needsRepatch = true;
                    projectedObject.set_transform_4by4(projected_position);
                    projectedObject.render();
                    projectedObject.draw_handle.FaceColor = 'green';
                    projectedObject.draw_handle.FaceAlpha = 0.5;

                    queue = nan(PATH_LENGTH, length(self.robot.model.links));
                    local_q = self.current_q;
                    for i = 1:PATH_LENGTH %Generate path with multiple RMRC
                        jac = self.robot.model.jacob0(local_q);
                        jaci = inv(jac);
                        qd = jaci * [self.tracked_object_velocity(1) self.tracked_object_velocity(2) -0.1 / PATH_LENGTH 0 0 0]';
                        qdt = qd';
                        local_q = local_q + qdt;
                        queue(i,:) = local_q;
                    end
                    self.present_queue_robot.add(queue);

                    self.state = 2;

                    end


                case 2
                    %STAGE 2: Close
                    %Once present queue is empty, emit gripper closing
                    %quickly while maintaining rubbish velocity. 
                    %tracked_object.attach_parent(self)
                    %Increment state
                    
                    %TODO
                    %We dont actually need to implement claw rendering at
                    %all, so this is probably fine as is
                    self.state = 3
                case 3
                    %STAGE 3: Pickup
                    %Once present queue is empty, emit path arm upwards to
                    %neutral position. 
                    %Increment state.
                    if self.present_queue_robot.is_empty
                        %find start velocity, same as what we
                        %did in case 1
                        self.tracked_object{1}.attach_parent(self);
                        PATH_LENGTH = self.path_lengths(4);
                        new_q = deg2rad([90,-90,0,-90,-90,0]);
                        jac = self.robot.model.jacob0(self.current_q);
                        jaci = inv(jac);
                        qd = jaci * [self.tracked_object_velocity(1) * PATH_LENGTH self.tracked_object_velocity(2) * PATH_LENGTH -0.1 / PATH_LENGTH 0 0 0]'
                        disp("------QD------")
                        qdt = qd';

                        %find starting qd with RMRC

                        trajectory = jtraj(self.current_q,new_q,PATH_LENGTH,qdt,[0 1 0 0 0 0]);
                    
                        self.present_queue_robot.add(trajectory);
                        self.state = 4;
                    end
                    %Generate trajectory to neutral position
                    
                case 4
                    %STAGE 4: Dropoff Positioning
                    %Once present queue is empty, emit path to move arm to
                    %rubbish chute opposite conveyor belt.
                    %Increment state.
                    if self.present_queue_robot.is_empty
                        PATH_LENGTH = self.path_lengths(5)
                        new_q = deg2rad([90,0,0,-90,-90,0]);
                        trajectory = jtraj(self.current_q,new_q,PATH_LENGTH,[0 1 0 0 0 0],[0 0 0 0 0 0]);
                        self.present_queue_robot.add(trajectory);
                        self.state = 5;
                    end
                case 5
                    %Stage 5: Release
                    %Once present queue is empty, release claw and orphan
                    %object.
                    %tracked_object.orphan();
                    %tracked_object = [];
                    %Increment state.
                    if self.present_queue_robot.is_empty
                        self.tracked_object{1}.orphan();
                        self.tracked_object{1} = [];
                        self.state = 6;
                    end
                case 6
                    %Stage 6: Return
                    %Once present queue is empty, emit path to neutral
                    %position (arm upwards)
                    %Set state to 0.
                    if self.present_queue_robot.is_empty
                        PATH_LENGTH = self.path_lengths(7);
                        new_q = deg2rad([90,-120,-60,-90,90,0]);
                        trajectory = jtraj(self.current_q,new_q,PATH_LENGTH);
                        self.present_queue_robot.add(trajectory);
                        self.state = 0;
                    end
                       

                        
                   
                case 7
                case 8
                case 1000
                    %Stage 1000: Substage of stage 0. Used to measure
                    %velocity of object and use this information to
                    %generate path to object

                    PATH_LENGTH = self.path_lengths(1);

                    self.tracked_object_info{2} = self.tracked_object{1}.current_transform;
                    self.tracked_object_velocity = self.tracked_object_info{2}(1:3,4) - self.tracked_object_info{1}(1:3,4);

                    %Generate path

                    
                    %project
                    projected_distance = self.tracked_object_velocity * PATH_LENGTH;
                    projected_position = self.tracked_object_info{2} * transl(projected_distance);
                    %render a rubbish here
                    projectedObject = copy(self.tracked_object{1});
                    projectedObject.needsRepatch = true;
                    projectedObject.set_transform_4by4(projected_position);
                    projectedObject.render();
                    projectedObject.draw_handle.FaceColor = 'green';
                    projectedObject.draw_handle.FaceAlpha = 0.5;

                    %disp(projected_position)

                    %ikine projected position

                    %Modify projected position to have rotation pointing down always and moved
                    %up slightly
                    projected_position(3,4) = projected_position(3,4) + 0.1;
                    projected_position(1:3,1:3) = eye(3);
                    projected_position(3,3) = -1; %pointing DOWN
                    moveto = self.robot.model.ikine(projected_position,'q0',self.robot.model.getpos,'mask',[1 1 1 1 1 0],'verbose=2')
                    

                    %generate RMRC to get proper joint velocity to smoothly
                    %move into downwards dropdown step

                    jac = self.robot.model.jacob0(self.robot.model.getpos())
                    jaci = inv(jac)
                    qd = jaci * [self.tracked_object_velocity(1) * PATH_LENGTH self.tracked_object_velocity(2) * PATH_LENGTH -0.02 0 0 0]'

                    trajectory = jtraj(self.robot.model.getpos,moveto,PATH_LENGTH,[0 0 0 0 0 0],qd');
                    self.present_queue_robot.add(trajectory);

                 
                    self.state = 1;


            end

            
            
            num_children = length(self.attached_child);
            

            for i = 1:num_children
                self.attached_child{i}.tick(); %tick children
            end

        end

        function render(self)
            self.detection_cubes(1).render();
            
            %disp("UR3EC: Render code stubbed");
            robot_q = self.present_queue_robot.pull();
            %clawQ = self.present_queue_claw.pull()

            self.robot.model.animate(robot_q);
            self.current_q = robot_q;
            %teleport attached children to end effector

            num_children = length(self.attached_child);
            c = self.robot.model.fkine(self.current_q);
            %ONLY move objects classified as "Rubbish" to the end effector!
            for i = 1:num_children 
                switch self.attached_child{i}.pc_type
                    case "Rubbish"
                        self.attached_child{i}.set_transform_4by4(c);
                end
            end

            
            %claw rendering code idk

            if ~isempty(self.attached_child)
                for i = 1:length(self.attached_child)
                    self.attached_child{i}.render(); %render children
                end
            end
            
        end

        
    end
end