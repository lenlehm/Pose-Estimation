function p = projectOntoImage(x, P_M, K) % Project 3D point onto image
% x = vector of model to camera pose parameters [ax, ay, az, tx, ty, tz]
% P_M = [X;Y;Z;1] input point
% K = intrinsic camera matrix
% p = output point [x,y]

% extract the pose parameters
ax = x(1); ay = x(2); az = x(3);
tx = x(4); ty = x(5); tz = x(6);

%Rotataion matrix, model to camera
Rx = [ 1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
Ry = [ cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)];
Rz = [ cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1];
R = Rz * Ry * Rx;

% Extrinsic camera Matrix
Mext = [ R [tx; ty; tz] ];

% Project Point
ph = K*Mext *P_M;

% For a single point
%ph= ph/ph(3); % get the x image and y image, because homogenoues coordinates used!
%p = ph(1:2);

ph(1, :) = ph(1, :)./ph(3, :);
ph(2, :) = ph(2, :)./ph(3, :);
ph = ph(1:2, :); % get rid of the 3rd row
p = reshape(ph, [], 1); % reshape into 2Nx1 vector
p = p(1:12); % have same length than the ground truth y0
end