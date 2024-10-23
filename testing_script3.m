clear all;

gg = light()
gg.Position = [-4, 0, 2];

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

drawnow();
axis equal;
view(3);
robot_ur3e = UR3EC(transl(-2,-0.4,0), dc);



%rate = rateControl(10);
for i = 1:100000
    conveyor_belt.tick();
    robot_ur3e.tick();
    conveyor_belt.render();
    robot_ur3e.render();
    drawnow;
    %waitfor(rate);
    
    axis equal
end