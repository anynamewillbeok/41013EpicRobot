img = imread('concrete.jpg');
img2 = imread('Recycling Background.jpg');
img3 = imread('BackgroundRecycle.jpg');

img2 = rot90(img2, 3);  % 270 rotation
img3 = rot90(img3, 2);  % 180 rotation

figure;
hold on;
axis equal;

% axis limit - if we need bigger room
xlim([0 10]);
ylim([0 10]);
zlim([0 10]);

% BASE (XY)
Z = zeros(size(X)); 
surface(X, Y, Z, 'CData', img, 'FaceColor', 'texturemap');
% ZY wall
X2 = zeros(size(Z2));
surface(X2, Y2, Z2, 'CData', img2, 'FaceColor', 'texturemap');
% ZX wall
Y3 = zeros(size(X3)); 
surface(X3, Y3, Z3, 'CData', img3, 'FaceColor', 'texturemap'); 

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

view(3);
grid off;
