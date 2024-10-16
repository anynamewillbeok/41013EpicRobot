clear all;

room_size = 10;
conveyor_start = [-2, 5, 0.5];
brick_offset_z = 0.5;

room = RoomPlotter('concrete.jpg', 'Recycling Background.jpg', 'BackgroundRecycle.jpg');
room.plotRoom();

gg = light();
gg.Position = [-4, 0, 2];

conveyor_belt = ConveyorBelt('ConveyorBeltFixed.PLY');
conveyor_belt.set_transform_4by4(transl(conveyor_start));

%initialize and position bricks
brick_positions = [-1.7, 4.95; -1.9, 4.85; -1.8, 5.15];  % X, Y positions of bricks
bricks = initialize_bricks(brick_positions, conveyor_belt, brick_offset_z);
axis equal;
view(3);

for i = 1:1000
    conveyor_belt.tick();
    conveyor_belt.render();
    pause(16.66 / 1000);
end

%function initialize and position bricks
function bricks = initialize_bricks(brick_positions, conveyor_belt, offset_z)
    num_bricks = size(brick_positions, 1);
    bricks = Rubbish.empty(num_bricks, 0);
    for i = 1:num_bricks
        bricks(i) = Rubbish('HalfSizedRedGreenBrick.ply');
        bricks(i).attach_parent(conveyor_belt);
        bricks(i).set_transform_4by4(transl(brick_positions(i, 1), brick_positions(i, 2), offset_z));
    end
end
