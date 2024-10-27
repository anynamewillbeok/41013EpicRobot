clear all;

view(3);
xlim([-3 0]);
ylim([-1.3 0.5]);
zlim([-0.5 1.3]);

figure(1);

drawnow();
conveyor_belt = ConveyorBelt('ConveyorBeltFixed.PLY');

conveyor_belt.set_transform_4by4(eye(4));

a = Rubbish('HalfSizedRedGreenBrick2.ply');
b = Rubbish('HalfSizedRedGreenBrick2.ply');
c = Rubbish('HalfSizedRedGreenBrick2.ply');

dc = DetectionController;
dc.register(a);
dc.register(b);
dc.register(c);

a.attach_parent(conveyor_belt);
a.set_transform_4by4(transl(0,0,0));

b.attach_parent(conveyor_belt);
b.set_transform_4by4(transl(0.3,-0.1,0));

c.attach_parent(conveyor_belt);
c.set_transform_4by4(transl(-0.2,0.1,0));

conveyor_belt.render();

light('Style','local','Position',[ -1.5 0 2],'Parent',gca);


drawnow;




ucc = UltimateCollisionChecker;
robot_ur3e = UR3EC(transl(-1.0,-0.4,0), dc, ucc);
robot_ur3e2 = UR3EC(transl(-1.5,-0.4,0), dc, ucc);
robot_ur3e3 = UR3EC(transl(-2.0,-0.4,0), dc, ucc);
robot_ur3e4 = UR3EC(transl(-2.5,-0.4,0), dc, ucc);
robot_abbc = ABBC(transl(-0.5,0.8,0) * trotz(pi),dc, ucc);
robot_abbc2 = ABBC(transl(-3.0,0.4,0) * trotz(pi + pi/4),dc, ucc);

robot_array = cell(1,6);
robot_array{1} = robot_ur3e;
robot_array{2} = robot_ur3e2;
robot_array{3} = robot_ur3e3;
robot_array{4} = robot_ur3e4;
robot_array{5} = robot_abbc;
robot_array{6} = robot_abbc2;

RandomBrickArray = createArray(0,0,'cell');

xlim([-6 6]);
ylim([-3 3]);
zlim([-0.5 3]);

hold on;
        conveyor_belt.render();
        robot_ur3e.render();
        robot_ur3e2.render();
        robot_ur3e3.render();
        robot_ur3e4.render();
        robot_abbc.render();
        robot_abbc2.render();
        hold off;

%rate = rateControl(30);
%profile on -timestamp -historysize 10000000 -timer performance

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
    stick = vrjoystick(1)
    robotarray{i}.total_control_activate(stick);
end



times = createArray(1,3000,"double");
i = 0;

for i = 1:3000
    timeStart = tic;
    %Every tick, random chance to spawn a new Rubbish
    if mod(i,50) == 0
        %Spawn Rubbish or Big Rubbish
        if rand > 0.25 %regular rubbish
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
    robot_ur3e.tick();
    robot_ur3e2.tick();
    robot_ur3e3.tick();
    robot_ur3e4.tick();
    robot_abbc.tick();
    robot_abbc2.tick();
    ucc.tick();
        hold on;
        conveyor_belt.render();
    robot_ur3e.render();
    robot_ur3e2.render();
    robot_ur3e3.render();
    robot_ur3e4.render();
    robot_abbc.render();
    robot_abbc2.render();
        hold off;
    %end

    drawnow;
    %waitfor(rate);
    times(i) = toc(timeStart);
    %times(i) = timeElapsed;
    end

figure(2);
plot(times);
%profile viewer