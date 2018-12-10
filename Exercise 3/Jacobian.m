clear all
close all
I = imread("C:\Users\Lenny\Documents\Studium_Robotics (M.Sc.)\Semester 1\Tracking and Detection in CV\exercise1\data\data\images\init_texture\DSC_9743.jpg");
imshow(I, []);
P_M = getpts;
f = 2960.37845;
cx = 1841.68855;
cy = 1235.23369;

K = [f 0 cx; 0 f cy; 0 0 1];

y0 = [ 183; 147;
    350; 133;
    454; 144;
    176; 258;
    339; 275;
    444; 286 ];

% initial guess of pose
x = [1.5; -1.0; 0.0; 0.0; 0.0; 30]
y = projectOntoImage(x, P_M, K)
for i=1:2:length(y)
    rectangle("Position", [y(i)-8 y(i+1)-8 16 16], "FaceColor", "r");
end

%% Jacobian
for i =1:10
    fprintf('\nIteration %d\nCurrent pose:\n', i);
    disp(x)
    % get predicted image points
    y = projectOntoImage(x,P_M,K);
    imshow(I, [])
    for i=1:2:length(y)
        rectangle("Position", [y(i)-8 y(i+1)-8 16 16], "FaceColor", "r");
    end
    pause(1);
    %Estimate Jacobian
    e = 0.000001;
    J(:, 1) = ( projectOntoImage(x+[e; 0; 0; 0; 0; 0], P_M, K) - y) / e;
    J(:, 2) = ( projectOntoImage(x+[0; e; 0; 0; 0; 0], P_M, K) - y) / e;
    J(:, 3) = ( projectOntoImage(x+[0; 0; e; 0; 0; 0], P_M, K) - y) / e;
    J(:, 4) = ( projectOntoImage(x+[0; 0; 0; e; 0; 0], P_M, K) - y) / e;
    J(:, 5) = ( projectOntoImage(x+[0; 0; 0; 0; e; 0], P_M, K) - y) / e;
    J(:, 6) = ( projectOntoImage(x+[0; 0; 0; 0; 0; e], P_M, K) - y) / e;
    
    dy = y0 - y;

    fprintf('residual error: %f\n', norm(dy));

    dx = pinv(J) * dy;
    if abs( norm(dx)/norm(x) ) < 1e-6
        break;
    end
    x = x+ dx;
end

%% Overlaying Graphical model
u0 = projectOntoImage(x, [0;0;0;1], K);
uX = projectOntoImage(x, [1;0;0;1], K); % unit X vector
uY = projectOntoImage(x, [0;1;0;1], K); % unit Y vector
uZ = projectOntoImage(x, [0;0;1;1], K); % unit Z vector

line([u0(1) uX(1)], [u0(2) uX(2)], 'Color', 'r', 'LineWidth', 3);
line([u0(1) uY(1)], [u0(2) uY(2)], 'Color', 'g', 'LineWidth', 3);
line([u0(1) uZ(1)], [u0(2) uZ(2)], 'Color', 'b', 'LineWidth', 3);