function [ rotationMatrix_inlier,translationVector_inlier] = Ransac( image,N,t )
%takes the image, the Number of iterations and the threshold for inliers as
%argument
% output is the rotation Matrix and Translation vector (Extrinsics) of the
% inlier points
% In the database: we picked the corner points of the image to have a 2D -
% 3D correspondence. We can obtain our 3D points by the teabox.ply 

%% Read the world  
[r,face] = read_ply('teabox.ply');
% load our model from the previous exercises 
result = load('result.mat'); % features of the database
result = result.result; % get the right data structure
% get the camera intrinsic
intrinst_m = [2960.37845 0 0;
                0  2960.37845 0;
                1841.68855 1235.23369 1];
cameraParams = cameraParameters('IntrinsicMatrix',intrinst_m);
% load the descriptors of the database
D = load('feature.mat');
D = D.D ;

% convert to gray, to feed it in the SIFT descriptor
X = rgb2gray(image);

peak_thresh = 2; % threshold to the SIFT: the higher, the less points
[fb,db] = vl_sift(single(X),'PeakThresh', peak_thresh); % get the keypoints in the image
[matches, scores] = vl_ubcmatch(D, db, 2); % compare descriptors to our database models

% get the corresponding world points and image points
world_point = result(matches(1,:),:);
image_point = fb(1:2,matches(2,:))';

% store the maximum number of inliers in here
Max = 0; 
% Count PnP error up
pnp_err = 0 ;

for i = 1:N
    % randomly select a point to check it to our correspondences
    perm = randperm(size(matches,2)) ;
    image_point_random =image_point( perm(1:4),:);
    world_point_random = world_point(perm(1:4),:);         

    try % get the the camera pose of our randomly selected points
        [worldOrientation,worldLocation] = estimateWorldCameraPose(image_point_random,world_point_random,cameraParams,'MaxReprojectionError',1);
    % If PnP doesn's find the solution, change the point
    catch ME
        % count our error up to see how many wrong randoms we have
        pnp_err = pnp_err + 1 ;
        continue;
    end
   % get the Extrinsics of our camera
    [rotationMatrix,translationVector] = cameraPoseToExtrinsics(worldOrientation,worldLocation);
%      P = cameraMatrix(cameraParams,rotationMatrix,translationVector);

%     estimate_point = [world_point ones([size(matches,2),1])] * P;
%     estimate_point(:,1) = estimate_point(:,1)./estimate_point(:,3);
%     estimate_point(:,2) = estimate_point(:,2)./estimate_point(:,3);
%     estimate_point(:,3) = [] ;

    % back projection to the image
    estimate_point = worldToImage(cameraParams,rotationMatrix,translationVector,world_point);

    err = estimate_point - image_point; % calculates the error
%     [eucdist, idx] = pdist2(estimate_point,image_point,'euclidean','smallest',1);
%     e = find(eucdist(1,idx) < 2);
    % Take the euclidiean distance of our backprojection to our image points
    eucdist = pdist2(estimate_point,image_point,'euclidean','smallest',1);
    e = find(eucdist < t);
    % store the largest number of inliers
    if size(e,2) > Max
        Max = size(e,2);
%         Ransac_worldOrientation = worldOrientation;
%         Ransac_worldLocation = worldLocation;
%         Ransac_P = P ;
        inlier = image_point(e,:); % store inlier points
    end
end
% get camera pose of our inlier points and camera extrinsics
[worldOrientation_inlier,worldLocation_inlier] = estimateWorldCameraPose(image_point(e,:),world_point(e,:),cameraParams);
[rotationMatrix_inlier,translationVector_inlier] = cameraPoseToExtrinsics(worldOrientation_inlier,worldLocation_inlier);
% backproject the points on the image
line_point = worldToImage(cameraParams,rotationMatrix_inlier,translationVector_inlier,r );

% plot the image and the bounding boxes
figure(7);
imshow(X);
hold on;
% plot(image_point(:,1),image_point(:,2),'r*','Color','r');
% plot(inlier(:,1),inlier(:,2),'r*','Color','g');
plot([line_point(1,1),line_point(2,1)],[line_point(1,2),line_point(2,2)],'LineWidth', 1, 'Color','b');
plot([line_point(3,1),line_point(4,1)],[line_point(3,2),line_point(4,2)],'LineWidth', 1, 'Color','b');
plot([line_point(5,1),line_point(6,1)],[line_point(5,2),line_point(6,2)],'LineWidth', 1, 'Color','b');
plot([line_point(7,1),line_point(8,1)],[line_point(7,2),line_point(8,2)],'LineWidth', 1, 'Color','b');
plot([line_point(2,1),line_point(3,1)],[line_point(2,2),line_point(3,2)],'LineWidth', 1, 'Color','b');
plot([line_point(1,1),line_point(4,1)],[line_point(1,2),line_point(4,2)],'LineWidth', 1, 'Color','b');
plot([line_point(6,1),line_point(7,1)],[line_point(6,2),line_point(7,2)],'LineWidth', 1, 'Color','b');
plot([line_point(5,1),line_point(8,1)],[line_point(5,2),line_point(8,2)],'LineWidth', 1, 'Color','b');
plot([line_point(4,1),line_point(8,1)],[line_point(4,2),line_point(8,2)],'LineWidth', 1, 'Color','b');
plot([line_point(3,1),line_point(7,1)],[line_point(3,2),line_point(7,2)],'LineWidth', 1, 'Color','b');
plot([line_point(2,1),line_point(6,1)],[line_point(2,2),line_point(6,2)],'LineWidth', 1, 'Color','b');
plot([line_point(1,1),line_point(5,1)],[line_point(1,2),line_point(5,2)],'LineWidth', 1, 'Color','b');

end
