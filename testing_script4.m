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

robot_ur3e = UR3EC(transl(-2.5,-0.4,0), dc);
robot_abbc = ABBC(transl(-0.5,0.8,0) * trotz(pi),dc);

RandomBrickArray = createArray(0,0,'cell');

xlim([-6 6]);
ylim([-3 3]);
zlim([-0.5 3]);

rate = rateControl(30);
for i = 1:100000  
    %Every tick, random chance to spawn a new Rubbish
    if mod(i,50) == 0
        %Spawn brick
        r = Rubbish('HalfSizedRedGreenBrick2.ply');
        r.attach_parent(conveyor_belt);
        RandomBrickArray{end+1} = r;
        dc.register(r);
        r.set_transform_4by4(transl(-0.1,((rand - 0.5) * 0.2),0));
    end
    conveyor_belt.tick();
    robot_ur3e.tick();
    robot_abbc.tick();
    conveyor_belt.render();
    robot_ur3e.render();
    robot_abbc.render();

    drawnow;
    waitfor(rate);
    
end