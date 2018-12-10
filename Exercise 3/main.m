%% Get an initial Pose hypothesis of the first frame with PnP and Ransac
% A = [2960.37845 0 1841.68855;
%      0 2960.37845 1235.23369;
%      0 0 1];
 A = [2960.37845 0 0;
     0 2960.37845 0;
     1841.68855 1235.23369 1];
cameraParams = cameraParameters('IntrinsicMatrix', A); % create the intrinsics camera
% get the 3D model
[r, face] = read_ply('teabox.ply');
face = face + 1;
n = 775; % 9775:9821
P = fullfile("tracking\",sprintf('DSC_9%d.JPG',n));
image = imread(P);
% figure();
% imshow(image);
% hold on;

% get the right image point corners 
%[x_corners, y_corners] = getpts;
% y0 =    1.0e+03 *  [1.2669;    1.3042; %top left corner
%                     1.1364;    1.2907; % top left corner depth (behind)
%                     2.1742;    1.2877; % top right corner (only one visible)
%                     2.1787;    1.8201; % bottom right corner
%                     1.2654;    1.9341; % bottom left corner
%                     1.1259;    1.8486 ]; % bottom left corner depth
                
% check whether the points fit:
% for i=1:2:length(y0)
%     rectangle("Position", [y0(i)-8 y0(i+1)-8 16 16], "FaceColor", "r");
% end

T = 2; % Threshold for RANSAC
N = 1000 ; % # Iterations for RANSAC
% get the RANSAC inlier Points of the first frame along with its Rotation
% and Translation Matrix/ vector
[ orientation, location ,R_inliers, T_inliers, image_point] = Ransac(image, N, T, cameraParams, r);

% Pose Refinement of initial guess using Jacobian
% transposing through: transpose(a) or a.'
% getting Mext of camera pose
M_ext = horzcat(R_inliers, transpose(T_inliers)); % horizontically stack the two Matrices/ vector
image_corners_estimate = image_point(:, :);
%plot(image_corners_estimate(:,1),image_corners_estimate(:,2),'r*','Color','r');

% Reshaping image corners to an ! column vector with [x1; y1; x2; y2; ..];
% corners = ones(length(image_corners_estimate), 1);
% j = 1;
% for i = 1 : length(image_corners_estimate)
%    corners(j) = image_corners_estimate(i); % x values
%    j = j + 1;
%    corners(j) = image_corners_estimate(i, 2); % y value
%    j = j + 1;
% end

rotationVector = rotationMatrixToVector(R_inliers); % Matlab toolbox - convert to exponential maps
x = horzcat(rotationVector, T_inliers); % x = vector (6cols, 1 row) of pose parameters
x = transpose(x); % get dimensions straight to add columnswise

% get world points of edges (in homogenoues) to compute Jacobian
[result, d] = image2world(r, face, image, R_inliers, T_inliers, cameraParams);
one = ones(length(result), 1); % convert to homogeneous coordinate system
result = horzcat(result, one);
y = worldToImage(cameraParams,R_inliers, T_inliers,result);
%y = projectOntoImage(x, transpose(result), A);
figure();
imshow(image);
hold on;

for i=1 : 2 : length(y)
    rectangle("Position", [y(i)-8 y(i+1)-8 16 16], "FaceColor", "r");
end
%% Jacobian 10 iterations
%J = ones(length(y0), length(x)); % initialize Jacobian
for i =1:10
    fprintf('\nIteration %d\nCurrent pose:\n', i);
    disp(x)
    % get predicted image points
    imshow(image)
    y = worldToImage(cameraParams,R_inliers, T_inliers,result);
    for i=1:2:length(y)
        rectangle("Position", [y(i)-8 y(i+1)-8 16 16], "FaceColor", "r");
    end
    pause(1);
    %Estimate Jacobian
    e = 0.000001;
    J(:, 1) = ( projectOntoImage(x+[e; 0; 0; 0; 0; 0], transpose(result), A) - y) / e;
    J(:, 2) = ( projectOntoImage(x+[0; e; 0; 0; 0; 0], transpose(result), A) - y) / e;
    J(:, 3) = ( projectOntoImage(x+[0; 0; e; 0; 0; 0], transpose(result), A) - y) / e;
    J(:, 4) = ( projectOntoImage(x+[0; 0; 0; e; 0; 0], transpose(result), A) - y) / e;
    J(:, 5) = ( projectOntoImage(x+[0; 0; 0; 0; e; 0], transpose(result), A) - y) / e;
    J(:, 6) = ( projectOntoImage(x+[0; 0; 0; 0; 0; e], transpose(result), A) - y) / e;
    
    dy = y0 - y;

    fprintf('residual error: %f\n', norm(dy));

    dx = pinv(J) * dy;
    disp(dx);
    if abs( norm(dx)/norm(x) ) < 1e-6
        break;
    end
    x = x + dx;
end

% %% get the points of the first frame
% n = 75;
% P = fullfile("C:\Users\Lenny\Documents\MATLAB\Exercise 3\tracking",sprintf('DSC_97%d.JPG',n));
% images{n} = imread(P);
% [RANSAC_Orientation,RANSAC_Location] = Ransac(images{n}, N, T);
% 
% % world point 1 
% [worldOrientation_1,worldLocation_1] = estimateWorldCameraPose(image_point_1,world_point_1,cameraParams,'MaxReprojectionError',2);
% [rotationMatrix,translationVector] = cameraPoseToExtrinsics(worldOrientation_1,worldLocation_1);
% % get the rotation and translation Vector
% rotationVector = rotationMatrixToVector(rotationMatrix)
% % call energy function
% result = energyFunction(rotationVector, translationVector, cameraParams, worldPoints, imagePoints)
% 
% % % Perform SIFT to obtain Rotation Matrix
% % X = rgb2gray(images{n});
% % peak_thresh = 2;
% % [fb,db] = vl_sift(single(X),'PeakThresh', peak_thresh);
% % %[RANSAC_Orientation,RANSAC_Location] = Ransac(images{n}, N, T);
% % % same shit for the second frame
% % n = n + 1;
% % P = fullfile("C:\Users\Lenny\Documents\MATLAB\Exercise 3\tracking",sprintf('DSC_97%d.JPG',n));
% % images{n} = imread(P);
% % X2 = rgb2gray(images{n});
% % peak_thresh = 2;
% % [fb,db] = vl_sift(single(X),'PeakThresh', peak_thresh);
% % % get the matches along with their scores
% % [matches, scores] = vl_ubcmatch(D, db, 3.6) ;
% % world_point = result(matches(1,:),:);
% % image_point = fb(1:2,matches(2,:))';
% 
% % rotationVector = rotationMatrixToVector(rotationMatrix) %returns an axis-angle rotation vector that corresponds to the input 3-D rotation matrix
% % 
% % imagePoints = worldToImage(cameraParams,rotationMatrix,translationVector,worldPoints) %returns the projection of 3-D world points into an image given camera parameters, the rotation matrix, and the translation vector.
% % 
% % for n = 9775:9821
% % end
% % 
% % % plot tracejctory
% % t = 0:0.1:10;
% % x = 5*cos(t);
% % y = 5*sin(t);
% % z = sin(t)+cos(t)+t;
% % plot3(x,y,z,'*r');
% % %if you want to animate it
% % for i=1:length(x)
% %   plot3(x(i),y(i),z(i),'*r');
% %   hold on;
% %   pause(0.01);
% % end