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
a.set_transform_4by4(transl(0, 0, 0));

b.attach_parent(conveyor_belt);
b.set_transform_4by4(transl(0.3, -0.1, 0));

c.attach_parent(conveyor_belt);
c.set_transform_4by4(transl(-0.2, 0.1, 0));

conveyor_belt.render();
light('Style', 'local', 'Position', [-1.5 0 2], 'Parent', gca);

drawnow;

% Initialize the robots with detection controller
ucc = UltimateCollisionChecker;
robot_ur3e = UR3EC(transl(-0.5, -0.4, 0), dc, ucc);
robot_ur3e2 = UR3EC(transl(-1.0, -0.4, 0), dc, ucc);
robot_ur3e3 = UR3EC(transl(-1.5, -0.4, 0), dc, ucc);
robot_ur3e4 = UR3EC(transl(-2.0, -0.4, 0), dc, ucc);

RandomBrickArray = createArray(0, 0, 'cell');

xlim([-6 6]);
ylim([-3 3]);
zlim([-0.5 3]);

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

times = createArray(1,3000,"double");
i = 0;

for i = 1:3000
    tickStart = tic;
    global eStop_triggered
    if eStop_triggered
        break;
    end

    % Every tick, with some chance to spawn a new Rubbish
    if mod(i, 50) == 0
        % Spawn either regular or big rubbish
        if rand > 0.5 % Regular rubbish
            r = Rubbish('HalfSizedRedGreenBrick2.ply');
            r.attach_parent(conveyor_belt);
            RandomBrickArray{end+1} = r;
            dc.register(r);
            r.set_transform_4by4(transl(-0.1, ((rand - 0.5) * 0.2), 0));
        else % Big rubbish
            r = Rubbish('Can.ply');
            r.pc_type = "BigRubbish";
            r.attach_parent(conveyor_belt);
            RandomBrickArray{end+1} = r;
            dc.register(r);
            r.set_transform_4by4(transl(-0.1, ((rand - 0.5) * 0.2), 0));
        end
    end

    % only updating is Estop is not triggered
    conveyor_belt.tick();
    robot_ur3e.tick();
    robot_ur3e2.tick();
    robot_ur3e3.tick();
    robot_ur3e4.tick();
    
    hold on;
    conveyor_belt.render();
    robot_ur3e.render();
    robot_ur3e2.render();
    robot_ur3e3.render();
    robot_ur3e4.render();
    hold off;
    drawnow;
    tickEnd = toc(tickStart);
    times(i) = tickEnd;
end

figure(2);
plot(times);

% closes UI
close(f);
