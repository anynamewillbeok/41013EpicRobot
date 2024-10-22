classdef UR3EC < handle & ParentChild & Tickable
    properties
        robot 
        present_queue_robot %Present queue holds
        present_queue_claw
        pc_type = "Robot"
        detection(1,1) DetectionController
        detection_cubes(1,:) DetectionCube
    end

    properties(SetAccess = private)
        state(1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(state, 256)}
        tracked_object
    end

    methods
        function self = UR3EC(transform, detection)
            self.robot = A2UR3E(transform);
            self.detection = detection;

            %starting Q positions
            %Q = [0,0,0]
            %self.robot.model.animate
            self.state = 0;

            %setup detection cube
            %transform 1: towards the conveyor belt idk where that is
            %really 
            cube1_transform = transform
            cube1_transform = cube1_transform * transl(0.5, 0.6 , 0);
            cube1_transform = cube1_transform * trscale(0.3, 0.5, 0.5);
            cube1 = DetectionCube("Cube.ply",detection,cube1_transform);
            self.detection_cubes(1) = cube1;
        end


        function tick(self)
            self.number_of_ticks = self.number_of_ticks + 1;

            switch self.state
                case 0
                    objects = self.detection_cubes(1).tick()
                    %STAGE 0: Detect rubbish. 
                    %Once detected, emit pathway to being above rubbish's projected
                    %position with ending velocity. Increment state.
                    %Tracked object is set to the detected object.
                case 1
                    %STAGE 1: Dropdown
                    %Once present queue is empty, maintaining rubbish sideways velocity, emit path move
                    %end effector downwards towards rubbish, while opening grabber.
                    %Increment state.
                case 2
                    %STAGE 2: Close
                    %Once present queue is empty, emit gripper closing
                    %quickly while maintaining rubbish velocity. 
                    %tracked_object.attach_parent(self)
                    %Increment state
                case 3
                    %STAGE 3: Pickup
                    %Once present queue is empty, emit path arm upwards to
                    %neutral position. 
                    %Increment state.
                case 4
                    %STAGE 4: Dropoff Positioning
                    %Once present queue is empty, emit path to move arm to
                    %rubbish chute opposite conveyor belt.
                    %Increment state.
                case 5
                    %Stage 5: Release
                    %Once present queue is empty, release claw and orphan
                    %object.
                    %tracked_object.orphan();
                    %tracked_object = [];
                    %Increment state.
                case 6
                    %Stage 6: Return
                    %Once present queue is empty, emit path to neutral
                    %position (arm upwards)
                    %Set state to 0.
                   
                case 7
                case 8
            end
        end

        function render(self)
            %disp("UR3EC: Render code stubbed");
            %robotQ = self.present_queue_robot.pull();
            %clawQ = self.present_queue_claw.pull()

            %self.robot.model.animate(robotQ);
            %claw rendering code idk
            
        end

        
    end
end