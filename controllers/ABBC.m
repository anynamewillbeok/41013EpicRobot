classdef ABBC < handle & ParentChild & Tickable
    properties
        robot
        base_transform;
        present_queue_robot FIFO %Present queues

        pc_type = "RobotABBCController"
        detection(1,1) DetectionController
        detection_cubes(1,:) DetectionCube

        path_lengths = [50,35,50,50,50,50,50,50];

        height_gap = 0.2 %Distance buffer to have between jtraj and RMRC when doing downwards motion

        projected_object

        home_position = deg2rad([90,-20,-20,0,60,0])

        dampening_max_lambda = 0.005;
        dampening_velocity_threshold = 0.15;

        %dco

        filter_type = "BigRubbish";
        linkdata(1,6)

        p_dc_t %Precomputed Detection Cube additional Transforms

        total_control(1,1) logical;
        handle_controller(1,1);
        handle_controller2;
        motion_axis_history FIFO;
        rotation_score(1,1);
        camera_transform_offsets = pi/4; %rotation around robot
        camera_height_offset = -pi/4;
    end

    properties(SetAccess = private)
        state(1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(state, 100000)}
        tracked_object
        tracked_object_info;
        tracked_object_velocity;
        current_q;
    end

    methods
        function self = ABBC(transform, detection, ultimate)
            %self.dco = detection;
            self.robot = ABB(transform);
            self.total_control = false;
            self.base_transform = transform;
            self.detection = detection;
            self.robot.model.animate(self.home_position);
            self.current_q = self.home_position;

            self.linkdata = self.robot.model.links;

            self.present_queue_robot = FIFO(length(self.robot.model.links), self);
            self.present_queue_robot.attach_parent(ultimate);
            self.present_queue_robot.force_add(self.home_position);


            %starting Q positions
            %Q = [0,0,0]
            %self.robot.model.animate
            self.state = 0;

            %setup detection cube
            %transform 1: towards the conveyor belt idk where that is
            %really
            cube1_transform = transform;
            cube1_transform = cube1_transform * transl(0, 0.8, 0.5);
            cube1_transform = cube1_transform * trscale(0.2, 0.6, 0.5);
            cube1 = DetectionCube(detection,cube1_transform);
            cube1.render();
            self.detection_cubes(1) = cube1;

            bin1_transform = transform;
            bin1_transform = bin1_transform * transl(0,-1.5,-1);
            bin1_transform = bin1_transform * trscale(1,1,1);
            bin = Bin("Bin.ply",detection, bin1_transform);
            bin.attach_parent(self);

            self.calculate_p_dc_t(self.linkdata);

            self.render();
        end

        function total_control_activate(self, controller_handle, controller_handle2)
            self.total_control = true;
            self.handle_controller = controller_handle;
            self.handle_controller2 = controller_handle2;
            %throw away FIFO data, we want control now!
            self.present_queue_robot.clear();
            self.motion_axis_history = FIFO(6, self);
            self.motion_axis_history.force_add(axis(controller_handle2));
            self.motion_axis_history.force_add(axis(controller_handle2));
            self.motion_axis_history.force_add(axis(controller_handle2));
            self.motion_axis_history.force_add(axis(controller_handle2));
            self.rotation_score = 0;
        end

        function total_control_deactivate(self)
            self.total_control = false;
        end

        function calculate_p_dc_t(self, links)
            self.p_dc_t = createArray(4,4,length(links),"double");
            thickness = [1 1.5 3 1 1 1];
            scale_y = [4 2.5 2.7 7 1 1];
            mask = [0 1 1 1 1 0];
            shift_z = [0 -1 -0.5 -0.15 -3 0];
            shift_x = [0 0 0 0 0 0];
            shift_y = [0 0 0 -0.25 0 0];
            scale_z = [1 1.25 2.7 0.2 1 1];
            scale_x = [4 1.25 2.7 0.2 1 1];
            num_links = length(links);

            scalelinks = copy(links);
            for i = 1:num_links
                if links(i).d == 0 & links(i).a == 0
                    scalelinks(i).d = 0.1;
                    scalelinks(i).a = 0.1;
                elseif links(i).d == 0
                    scalelinks(i).d = links(i).a * thickness(i);
                elseif links(i).a == 0
                    scalelinks(i).a = links(i).d * thickness(i);
                end
                self.p_dc_t(:,:,i) = transl((-links(i).a)/2,0,links(max([i-1 1])).d * mask(i) * 0.4) * trscale(abs(scalelinks(i).a) * 1.2 * scale_x(i),scale_y(i) * 0.1,abs(scalelinks(i).d) * 1.2 * scale_z(i)) * transl(shift_x(i),shift_y(i),0.5 * mask(i) + shift_z(i));
            end
        end


        function tick(self)
            self.number_of_ticks = self.number_of_ticks + 1;

            if ~self.total_control;

                switch self.state
                    case 0
                        if self.present_queue_robot.is_empty
                            detected_objects = self.detection_cubes(1).tick();
                            if ~isempty(detected_objects)
                                for i = 1:length(detected_objects)
                                    if detected_objects{i}.pc_type == self.filter_type
                                        disp("Detected Big Rubbish!");
                                        self.tracked_object = detected_objects(i);
                                        self.tracked_object_info{1} = self.tracked_object{1}.current_transform;
                                        self.state = 1000;
                                    end
                                end

                            end
                        end
                        %STAGE 0: Detect Big rubbish.
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

                        DLS_TRIGGERED = 0;

                        if self.present_queue_robot.is_empty
                            delete(self.projected_object.draw_handle);
                            delete(self.projected_object);



                            PATH_LENGTH = self.path_lengths(2);
                            %Generate path
                            %project
                            projected_distance = self.tracked_object_velocity * PATH_LENGTH;
                            projected_position = self.tracked_object{1}.current_transform * transl(projected_distance);
                            %render a rubbish here
                            projectedObject = copy(self.tracked_object{1});
                            projectedObject.pc_type = "Ghost";
                            projectedObject.needsRepatch = true;
                            projectedObject.set_transform_4by4(projected_position);
                            projectedObject.render();
                            projectedObject.draw_handle.FaceColor = 'green';
                            projectedObject.draw_handle.FaceAlpha = 0.5;
                            self.projected_object = projectedObject;

                            queue = nan(PATH_LENGTH, length(self.robot.model.links));
                            queue_dls = nan(PATH_LENGTH, length(self.robot.model.links));
                            local_q = self.current_q;
                            local_q_dls = self.current_q;
                            %This method can run into a pretty nasty singularity if
                            %the shaft manages to go completely vertical, so try to
                            %avoid with DLS

                            %
                            for i = 1:PATH_LENGTH %Generate path with multiple RMRC
                                jac = self.robot.model.jacob0(local_q);
                                jaci = inv(jac);
                                xdot = [self.tracked_object_velocity(1) self.tracked_object_velocity(2) -self.height_gap / PATH_LENGTH 0 0 0]';
                                qd = jaci * xdot;
                                qd_dls = qd;
                                maximum = max(abs(qd([end end-2]))); %Here, we're only really worried about the last and third last joints counter-rotating like crazy at the singularity, which is observed in the simulator
                                if maximum > self.dampening_velocity_threshold
                                    %Levenberg–Marquardt algorithm also known as
                                    %DLS
                                    DLS_TRIGGERED = 1;
                                    manipulability_score = sqrt(det(jac * jac'));
                                    dampening_lambda = (1 - (manipulability_score / self.dampening_velocity_threshold)^2);
                                    qd_dls = inv((jac' * jac + dampening_lambda * eye(length(self.robot.model.links)))) * jac * xdot; %Taken from 4 Damped Least Squares video 6:35
                                    %warning("ABBC is near singularity: generated joint velocity maximum at " + maximum + " over a single tick, attempting to resolve with DLS");
                                    %warning("DLS: qd maximum is " + max(abs(qd_dls)));
                                    %end DLS
                                end
                                local_q = local_q + qd';
                                local_q_dls = local_q_dls + qd_dls';
                                queue(i,:) = local_q;
                                queue_dls(i,:) = local_q_dls;



                            end
                            if DLS_TRIGGERED
                                warning("ABBC: Damped Least Squares triggered to avoid singularity, end-effector position affected!");

                                % debugfigure = figure(2);
                                % ax = axes(debugfigure);
                                %
                                % q = plot(ax, queue,'LineWidth',5);
                                % [q(:,1).Color] = deal('red');
                                % hold(ax, 'on');
                                % qdls = plot(ax, queue_dls,'LineWidth',5);
                                % [qdls(:,1).Color] = deal('blue');
                                % L(1) = plot(ax, nan, nan, 'r-');
                                % L(2) = plot(ax, nan, nan, 'b--');
                                % legend(ax, L, {"no DLS","yes DLS"}); %make fake legend data but it actually looks real
                                % ylabel(ax, "Q position");
                                % xlabel(ax, "Ticks during RMRS stage");
                                % drawnow
                                %
                                %
                                %
                                % hold(ax,'off');
                                % ax.HandleVisibility = 'off';
                                % figure(1);


                                % idk how to make this figure not make the
                                % main figure break

                                al = width(queue);
                                queue(:,[al al-2]) = queue_dls(:,[al al-2]); %swap out queue for DLS'd queue before submission for links end and end-2
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
                        self.state = 3;
                    case 3
                        %STAGE 3: Pickup
                        %Once present queue is empty, emit path arm upwards to
                        %neutral position.
                        %Increment state.
                        if self.present_queue_robot.is_empty

                            delete(self.projected_object.draw_handle);
                            delete(self.projected_object);

                            drawnow
                            %find start velocity, same as what we
                            %did in case 1
                            self.tracked_object{1}.attach_parent(self);
                            PATH_LENGTH = self.path_lengths(4);
                            new_q = self.home_position;
                            jac = self.robot.model.jacob0(self.current_q);
                            jaci = inv(jac);
                            qd = jaci * [self.tracked_object_velocity(1) * PATH_LENGTH self.tracked_object_velocity(2) * PATH_LENGTH 0 0 0 0]';

                            %disp("------QD------");
                            qdt = qd';

                            %find starting qd with RMRC

                            trajectory = jtraj(self.current_q,new_q,PATH_LENGTH,qdt,[0 0 0 0 0 0]);

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
                            PATH_LENGTH = self.path_lengths(5);

                            %find attached child bin
                            bin_found = false;
                            for i = 1:length(self.attached_child)
                                if self.attached_child{i}.pc_type == "Bin"
                                    bin_location = self.attached_child{i}.current_transform;
                                    bin_found = true;
                                    break
                                end
                            end
                            if ~bin_found
                                error("ABBC: Bin not found!");
                            end
                            target_location = bin_location * transl(0,0,1); %we want the target to be above the bin
                            target_location = target_location * trotx(pi); %and also have the end effector Z reversed
                            %scale target_location rotation vectors to have
                            %length 1
                            for i = 1:3
                                target_location(1:3,i) = target_location(1:3,i)/norm(target_location(1:3,i));
                            end

                            disp(target_location);
                            %generate ikine to bin
                            new_q = self.robot.model.ikine(target_location,'q0',deg2rad([-90,54,-81,0,0,0]),'mask',[1 1 1 1 1 1],'forceSoln');
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
                            new_q = self.home_position;
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
                        projectedObject.pc_type = "Ghost";
                        projectedObject.needsRepatch = true;
                        projectedObject.set_transform_4by4(projected_position);
                        projectedObject.render();
                        projectedObject.draw_handle.FaceColor = 'green';
                        projectedObject.draw_handle.FaceAlpha = 0.5;

                        %disp(projected_position)

                        %ikine projected position

                        %Modify projected position to have rotation pointing down always and moved
                        %up slightly
                        projected_position(3,4) = projected_position(3,4) + self.height_gap;
                        projected_position(1:3,1:3) = eye(3); %%reset rotation of projected brick position
                        projected_position = projected_position * trotx(pi); %pointing DOWN
                        moveto = self.robot.model.ikine(projected_position,'q0',deg2rad([90 50 -140 90 0 0]),'mask',[1 1 1 1 1 1],'verbose=2');


                        %generate RMRC to get proper joint velocity to smoothly
                        %move into downwards dropdown step

                        jac = self.robot.model.jacob0(self.robot.model.getpos());
                        jaci = inv(jac);
                        qd = jaci * [self.tracked_object_velocity(1) * PATH_LENGTH self.tracked_object_velocity(2) * PATH_LENGTH (-self.height_gap / self.path_lengths(2)) * PATH_LENGTH 0 0 0]';

                        trajectory = jtraj(self.robot.model.getpos,moveto,PATH_LENGTH,[0 0 0 0 0 0],qd');
                        self.present_queue_robot.add(trajectory);

                        self.projected_object = projectedObject;
                        self.state = 1;
                end
            else

                %TOTAL CONTROL: controller time!

                %DUALSENSE controller on LINUX:
                %Button 1:  X
                %Button 2:  CIRCLE
                %Button 3:  TRIANGLE
                %Button 4:  SQUARE
                %Button 5:  LEFT    BUMPER
                %Button 6:  RIGHT   BUMPER
                %Button 7:  LEFT    TRIGGER (20% PULL)
                %Button 8:  RIGHT   TRIGGER (20% PULL)
                %Button 9:  CREATE  (LEFT OF TRACKPAD)
                %Button 10: OPTIONS (RIGHT OF TRACKPAD)
                %Button 12: LEFT_STICK_PUSH
                %Button 13: RIGHT_STICK_PUSH

                %Axis 1: LEFT_STICK_X LEFT-NEGATIVE
                %Axis 2: LEFT_STICK_Y UP-NEGATIVE
                %Axis 3: LEFT_TRIGGER (UNPULLED -1, PULLED 1)
                %Axis 4: RIGHT_STICK_X LEFT-NEGATIVE
                %Axis 5: RIGHT_STICK_Y UP-NEGATIVE
                %Axis 6: RIGHT_TRIGGER (UNPULLED -1, PULLED 1)
                %Axis 7: LEFT -1 RIGHT 1 DPAD
                %Axis 8: UP   -1 DOWN  1 DPAD

                %

                [axes, buttons, povs] = read(self.handle_controller);
                [motion_axes, buttons2, povs2] = read(self.handle_controller2);
                self.motion_axis_history.force_add(motion_axes);
                motion_axes = self.motion_axis_history.get_active_and_queue(self.motion_axis_history.get_active_and_queue_length());
                self.motion_axis_history.pull();
                %filter data
                motion_axes = mean(motion_axes);

                CAMERA_ANGLE = self.camera_transform_offsets;
                CAMERA_ANGLE = CAMERA_ANGLE + (axes(7) / 25);
                self.camera_transform_offsets = CAMERA_ANGLE;

                CAMERA_ANGLE_HEIGHT = self.camera_height_offset;
                CAMERA_ANGLE_HEIGHT = CAMERA_ANGLE_HEIGHT + (-axes(8) / 25);
                self.camera_height_offset = CAMERA_ANGLE_HEIGHT;


                filtered_inputs = createArray(1,6,"double");

                if abs(axes(1)) > 0.2
                    filtered_inputs(1) = axes(1)^3;
                end

                if abs(axes(2)) > 0.2
                    filtered_inputs(2) = axes(2)^3;
                end

                if abs(axes(4)) > 0.2
                    filtered_inputs(4) = axes(4)^3;
                end

                if abs(axes(5)) > 0.2
                    filtered_inputs(5) = axes(5)^3;
                end

                if axes(3) > -0.9
                    %scale from (-1)-(1) to 0-1
                    filtered_inputs(3) = (axes(3) + 1) / 2;
                end
                if axes(6) > -0.9
                    filtered_inputs(6) = (axes(6) + 1) / 2;
                end
                %disp(filtered_inputs);

                [current_position, others] = self.robot.model.fkine(self.current_q);
                current_position = current_position.T;
                %others = others.T;
                inv2 = inv(self.base_transform);

                relative = inv2 * current_position;

                %remove rotation
                relative(1:3, 1:3) = self.base_transform(1:3,1:3);
                relative = relative * trotz(-CAMERA_ANGLE);

                %filter motion input
                %axis 1 is roll axis (flat is 0, anticlockwise is positive
                %until around 0.250 where controller is on its side)
                %axis 2 is "up" (trackpad normal, 0.255 pointing up, 0 if pointing to the side, -0.250 pointing down)
                %axis 3 is pitch (arrow pointing towards you, if its
                %pointing down its negative)

                %axis 4 is pitch speed (rolling forwards is negative)
                %axis 5 is turn speed (controller spinning around on table
                %flat) (anticlockwise positive)
                %axis 6 is roll speed (anticlockwise positive

                disp(motion_axes);

                self.rotation_score = self.rotation_score + motion_axes(5);


                %target_position = eye(4);
                %target_position(1:3,4) = current_position(1:3,4);
                %rotationTransform = eye(4) * trotx(motion_axes(1)) * troty(motion_axes(3));
                %rotation must be done relative to link end-1

                target_position = self.base_transform * trotz((filtered_inputs(3) - filtered_inputs(6)) / 50) * relative * transl(filtered_inputs(1)/200,-filtered_inputs(2)/200,-filtered_inputs(5)/200);% * trotz((buttons(5) - buttons(6)) * 0.04);% * rotationTransform;% * trotz((filtered_inputs(3) - filtered_inputs(6)) / 50);
                %clear rotation vectors otherwise motion controls will make
                %no sense
                %tilt axis
                %target_position(1:3,1:3) = eye(3);
                target_position(1:3,1:3) = rotz(-CAMERA_ANGLE) * rotx(pi);

                % upsidedown = -1;
                % if motion_axes(2) < 0
                %     %motion_axes(2) is negative if the controller isnt pointing
                %     %upwards
                %     upsidedown = 1;
                %     %target_position(1:3,3) = [0 0 1];
                % end
                % target_position = target_position * trotx(((upsidedown - 1) / 2) * pi)

                target_position = target_position * troty((motion_axes(1) + motion_axes(6)) * 1.5) * trotx((-motion_axes(3) + motion_axes(4)) * 1.5) * trotz(-CAMERA_ANGLE);

                %target_position = target_position * trotz((filtered_inputs(3) - filtered_inputs(6)) / 200) * transl(0,-filtered_inputs(1)/200,-filtered_inputs(2)/200);
                %target_position(1:3,1:3) = current_position(1:3,1:3);
                new_q = self.robot.model.ikine(target_position,'q0',self.current_q,'forceSoln');
                %new_q(1) = new_q(1) + (filtered_inputs(3) / 10);

                %CAMERA HACK


                cameraTransform = target_position;
                %throw away rotation data, keep camera axis fixed
                cameraTransform(1:3, 1:3) = eye(3) * rotz(-CAMERA_ANGLE) * rotx(CAMERA_ANGLE_HEIGHT);
                cameraTransform = cameraTransform * transl(0, -1, 0);% * trotx(0.6);


                camPos = cameraTransform(1:3, 4)';
                camTarget = cameraTransform * transl(0,1,0);
                camTarget = camTarget(1:3, 4).';
                camUpVec = cameraTransform(1:3, 3)'; %up vector is wherever Z is pointing

                h = gca;
                camproj('perspective');
                camva('manual');
                camva(60);
                set(h, 'CameraPosition', camPos, ...
                    'CameraTarget', camTarget, ...
                    'CameraUpVector', camUpVec);

                %generate detection hull, use to sample grabbing

                %check if we have any attached Rubbish
                attached_brick_child_id = 0;
                for i = 1:length(self.attached_child)
                    if self.attached_child{i}.pc_type == self.filter_type
                        attached_brick_child_id = i;
                    end
                end

                if buttons(1) && attached_brick_child_id == 0
                    boxes = self.build_detection_cubes(new_q);
                    boxes{6}.dco = self.detection;
                    detected_objects = boxes{6}.tick();
                    if ~isempty(detected_objects)
                        for i = 1:length(detected_objects)
                            if detected_objects{i}.pc_type == self.filter_type
                                %disp("Detected Regular Rubbish!");
                                self.tracked_object = detected_objects(i); %select first object found
                                self.tracked_object{1}.attach_parent(self);
                                break;
                            end
                        end
                    end
                end

                if buttons(2) && attached_brick_child_id ~= 0
                    self.tracked_object{1}.orphan();
                    self.tracked_object{1} = [];
                end

                % local_q = self.current_q;
                %
                % jac = self.robot.model.jacob0(local_q);
                % jaci = inv(jac);
                % xdot = [axes(1)/10 axes(2)/10 0 0 0 0]';
                % qd = jaci * xdot;

                %maximum = max(abs(qd));
                % if maximum > self.dampening_velocity_threshold
                %     %Levenberg–Marquardt algorithm also known as
                %     %DLS
                %     %DLS_TRIGGERED = 1;
                %     manipulability_score = sqrt(det(jac * jac'));
                %     dampening_lambda = (1 - (manipulability_score / self.dampening_velocity_threshold)^2);
                %     qd = inv((jac' * jac + dampening_lambda * eye(length(self.robot.model.links)))) * jac * xdot; %Taken from 4 Damped Least Squares video 6:35
                %     %warning("ABBC is near singularity: generated joint velocity maximum at " + maximum + " over a single tick, attempting to resolve with DLS");
                %     %warning("DLS: qd maximum is " + max(abs(qd_dls)));
                %     %end DLS
                % end
                %local_q = local_q + qd'
                %disp(new_q);
                self.present_queue_robot.force_replace(new_q);

            end










            num_children = length(self.attached_child);


            for i = 1:num_children
                self.attached_child{i}.tick(); %tick children
            end

        end


        function render(self)
            %self.detection_cubes(1).render();
            %disp("UR3EC: Render code stubbed");
            robot_q = self.present_queue_robot.pull();
            %clawQ = self.present_queue_claw.pull()
            %teleport attached children to end effector
            %ONLY move objects classified as "Rubbish" to the end effector!
            if robot_q ~= self.current_q | self.number_of_ticks == 1  %only animate if needs to be, or rendering for first time
                self.robot.model.animate(robot_q);
                c = self.robot.model.fkine(robot_q);
                if ~isempty(self.attached_child)
                    for i = 1:length(self.attached_child)
                        if self.attached_child{i}.pc_type == self.filter_type
                            self.attached_child{i}.set_transform_4by4(c); %teleport filtered objects to end effector
                        end
                        self.attached_child{i}.render(); %render children
                    end
                end
                self.current_q = robot_q;
            end
        end

        function boxes = build_detection_cubes(self, q_row)
            boxes = cell(1,6);
            num_links = length(self.linkdata);
            %[~, transforms] = self.robot.model.fkine(q);
            %reimplement fkine here to get current transforms
            transforms = createArray(4,4,width(q_row),"double");

            transformation = self.base_transform;

            function transform = a_copy(link, q)
                sa = sin(link.alpha);
                ca = cos(link.alpha);
                if link.flip
                    q = -q + link.offset;
                else
                    q = q + link.offset;
                end

                % if link.isrevolute
                % revolute (always)
                st = sin(q); ct = cos(q);
                d = link.d;
                % else
                %     % prismatic
                %     st = sin(link.theta); ct = cos(link.theta);
                %     d = q;
                % end

                %if L.mdh == 0
                % standard DH, always using standard DH so we can cut out
                % an if else statement for modified DH

                transform = [    ct  -st*ca  st*sa   link.a*ct
                    st  ct*ca   -ct*sa  link.a*st
                    0   sa      ca      d
                    0   0       0       1];
                %end
            end

            dc = DetectionController; %fake controller

            for i = 1:num_links
                transformation = transformation * a_copy(self.linkdata(i), q_row(i));
                transforms(:,:,i) = transformation;
                boxes{i} = DetectionCube(dc, transforms(:,:,i) * self.p_dc_t(:,:,i));
            end

            %transforms should be defined somewhere BEFORE this
            %transforms = transforms.T;
            %scale transforms
            %scalelinks = copy(links);
            for i = 1:num_links


                %= DetectionCube(dc,transforms(:,:,i));
            end
        end
    end


    methods(Static)

        function boxes = build_detection_cubes_static(q_row, links, base_transform, p_dc_t)

            boxes = cell(1,6);
            %links = self.robot.model.links;
            num_links = length(links);
            %[~, transforms] = self.robot.model.fkine(q); reimplement fkine
            %here
            transformation = base_transform;
            transforms = createArray(4,4,width(q_row),"double");

            function transform = a_copy(link, q) %copy of Link.A function because SerialLink hates being parallelised
                sa = sin(link.alpha);
                ca = cos(link.alpha);
                if link.flip
                    q = -q + link.offset;
                else
                    q = q + link.offset;
                end

                % if link.isrevolute
                % revolute (always)
                st = sin(q); ct = cos(q);
                d = link.d;
                % else
                %     % prismatic
                %     st = sin(link.theta); ct = cos(link.theta);
                %     d = q;
                % end

                %if L.mdh == 0
                % standard DH, always using standard DH so we can cut out
                % an if else statement for modified DH

                transform = [    ct  -st*ca  st*sa   link.a*ct
                    st  ct*ca   -ct*sa  link.a*st
                    0   sa      ca      d
                    0   0       0       1];
                %end
            end

            for i = 1:num_links
                transformation = transformation * a_copy(links(i), q_row(i));
                transforms(:,:,i) = transformation;
            end

            dc = DetectionController; %fake controller

            for i = 1:num_links
                boxes{i} = DetectionCube(dc, transforms(:,:,i) * p_dc_t(:,:,i));
            end
        end


    end
end