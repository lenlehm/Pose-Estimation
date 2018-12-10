function [ worldOrientation_inlier, worldLocation_inlier , rotationMatrix_inlier,translationVector_inlier, line_point ] = Ransac( image, N, t, cameraParams, r )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
result = load('result.mat');
result = result.result;
D = load('feature.mat');
D = D.D ;

X = rgb2gray(image);

peak_thresh = 2;
[fb,db] = vl_sift(single(X),'PeakThresh', peak_thresh);
[matches, scores] = vl_ubcmatch(D, db, 3.6) ;

world_point = result(matches(1,:),:);
image_point = fb(1:2,matches(2,:))';

Max = 0; 

pnp_err = 0 ;

for i = 1:N
    perm = randperm(size(matches,2)) ;
    image_point_random =image_point( perm(1:4),:);

    world_point_random = world_point(perm(1:4),:) ;         

    try 
        [worldOrientation,worldLocation,status] = estimateWorldCameraPose(image_point_random,world_point_random,cameraParams);
    % If PnP doesn's find the solution, change the point
    catch ME
        pnp_err = pnp_err +1 ;
        continue;
   end
    [rotationMatrix,translationVector] = cameraPoseToExtrinsics(worldOrientation,worldLocation);
     P = cameraMatrix(cameraParams,rotationMatrix,translationVector);

%     estimate_point = [world_point ones([size(matches,2),1])] * P;
%     estimate_point(:,1) = estimate_point(:,1)./estimate_point(:,3);
%     estimate_point(:,2) = estimate_point(:,2)./estimate_point(:,3);
%     estimate_point(:,3) = [] ;
    estimate_point = worldToImage(cameraParams,rotationMatrix,translationVector,world_point);
    %err = estimate_point - image_point;
    [eucdist, idx] = pdist2(estimate_point,image_point,'euclidean','smallest',1);
    e = find(eucdist(1,idx) < 2);
    if size(e,1) > Max
        Max = size(e,1);
%         Ransac_worldOrientation = worldOrientation;
%         Ransac_worldLocation = worldLocation;
%         Ransac_P = P ;
        inlier = image_point(e,:);
    end
end
[worldOrientation_inlier,worldLocation_inlier] = estimateWorldCameraPose(image_point(e,:),world_point(e,:),cameraParams);
[rotationMatrix_inlier,translationVector_inlier] = cameraPoseToExtrinsics(worldOrientation_inlier,worldLocation_inlier);

line_point = worldToImage(cameraParams,rotationMatrix_inlier,translationVector_inlier, r); % 3D world points projected on image

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
