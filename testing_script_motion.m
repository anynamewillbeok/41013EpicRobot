clear all;

%% INITALISE SCENE: create robots, conveyor belt, rubbish array and supporting elements

view(3);

figure(1);

drawnow();
conveyor_belt = ConveyorBelt('ConveyorBeltFixed.PLY');

conveyor_belt.set_transform_4by4(eye(4));

a = Rubbish('HalfSizedRedGreenBrick2.ply');
b = Rubbish('HalfSizedRedGreenBrick2.ply');
c = Rubbish('HalfSizedRedGreenBrick2.ply');

dc = DetectionController;

conveyor_belt.render();

light('Style','local','Position',[ -1.5 0 2],'Parent',gca);

drawnow;

ucc = UltimateCollisionChecker;

robot_array = cell(1,1);
robot_array{1} = UR3EC(transl(-1.0,-0.4,0), dc, ucc);
robot_array{end+1} = UR3EC(transl(-1.5,-0.4,0), dc, ucc);
robot_array{end+1} = UR3EC(transl(-2.0,-0.4,0), dc, ucc);
robot_array{end+1} = UR3EC(transl(-2.5,-0.4,0), dc, ucc);
robot_array{end+1} = ABBC(transl(-0.5,1.2,-0.5) * trotz(pi), dc, ucc);
robot_array{end+1} = ABBC(transl(-3.0,-1.2,-0.5),dc, ucc);

RandomBrickArray = createArray(0,0,'cell');

xlim([-6 0]);
ylim([-3 2]);
zlim([-0.5 2]);

% hold on;
%         conveyor_belt.render();
%         robot_ur3e.render();
%         robot_ur3e2.render();
%         robot_ur3e3.render();
%         robot_ur3e4.render();
%         robot_abbc.render();
%         robot_abbc2.render();
%         hold off;

%rate = rateControl(30);
%profile on -timestamp -historysize 10000000 -timer performance

%% UI & E-STOP

global eStop_triggered
eStop_triggered = false;

% UI BUTTON
f = uifigure;
stopButton = uibutton(f, 'Text', 'Emergency Stop', ...
    'Position', [20, 20, 150, 30], ...
    'ButtonPushedFcn', @(~, ~) triggerEStop());

% emergency stop flag
function triggerEStop()
    global eStop_triggered
    eStop_triggered = true;
end

% UI BUTTONS: TOTAL CONTROL

tcArray = cell(1,length(robot_array));

for i = 1:length(robot_array)
    tcArray{i} = uibutton(f, ...
    'Text', 'TOTAL CONTROL (' + robot_array{i}.pc_type + ')', ...
    'Position', [20, 30 + (i * 30), 300, 30], ...
    'ButtonPushedFcn', @(~, ~) triggerTotalControl(i, robot_array));
end

function triggerTotalControl(i, robotarray)
    stick = vrjoystick(1);
    stick2 = vrjoystick(2); %%Dualsense motion controls, could be funny!
    robotarray{i}.total_control_activate(stick, stick2);
end

%% RUN SIMULATION

times = createArray(1,3000,"double");
i = 0;

for i = 1:3000
    timeStart = tic;
    %Every tick, random chance to spawn a new Rubbish
    if mod(i,50) == 0
        %Spawn Rubbish or Big Rubbish
        if rand > 0.2 %regular rubbish
            r = Rubbish('HalfSizedRedGreenBrick2.ply');
            r.attach_parent(conveyor_belt);
            RandomBrickArray{end+1} = r;
            dc.register(r);
            r.set_transform_4by4(transl(-0.1,((rand - 0.5) * 0.2),0));
        else %big rubbish
            r = Rubbish('Can.ply');
            r.pc_type = "BigRubbish";
            r.attach_parent(conveyor_belt);
            RandomBrickArray{end+1} = r;
            dc.register(r);
            r.set_transform_4by4(transl(-0.1,((rand - 0.5) * 0.2),0));
        end

    end

    %if (~estop_triggered)
    conveyor_belt.tick();
    for j = 1:length(robot_array)
        robot_array{j}.tick();
    end
    ucc.tick();
    hold on;
    conveyor_belt.render();
    for j = 1:length(robot_array)
        robot_array{j}.render();
    end
    hold off;


    %end

    drawnow;
    %waitfor(rate);
    time = toc(timeStart);
    times(i) = time;
end

figure(5);
plot(times);
%profile viewer