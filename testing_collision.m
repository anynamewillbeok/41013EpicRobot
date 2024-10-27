clear all;

view(3);
xlim([-3 0]);
ylim([-1.3 0.5]);
zlim([-0.5 1.3]);

figure(1);

drawnow();

light('Style','local','Position',[ -1.5 0 2],'Parent',gca);


drawnow;

dc = DetectionController();
ucc = UltimateCollisionChecker;
robot_ur3e = ABBC(transl(-0.5,-0.4,0), dc, ucc);

RandomBrickArray = createArray(0,0,'cell');

%rate = rateControl(30);
boxes = robot_ur3e.build_detection_cubes(robot_ur3e.current_q);
% 
% for i = 1:length(boxes)
%     dhandle = boxes{i}.render();
%     dhandle.FaceAlpha = (i / 6);
% end
