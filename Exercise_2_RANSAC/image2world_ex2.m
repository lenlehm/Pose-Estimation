function [ result, d ] = image2world(r, face,image, rotationMatrix,translationVector, cameraParams, lamda)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%  [rotationMatrix,translationVector] = extrinsics(image_point,world_point,cameraParams);
 P = cameraMatrix(cameraParams,rotationMatrix,translationVector);
 p = P';
 Q = p(:,1:3); q = p(:,4);
 Orig = -inv(Q) * q ;


I = rgb2gray(image) ;
% imshow(I);
% hold on ;

peak_thresh = 2;
[f,d] = vl_sift(single(I),'PeakThresh', peak_thresh) ;
%h1 = vl_plotframe(f) ;
%set(h1,'color','b','linewidth',3) ;


result=[];
for i=1:size(f,2)
    % ray equation
    ray = Orig +  inv(lamda * Q) * [f(1:2,i)' 1]'; 
    % get the intersection of the the ray with the world teabox
    % (teabox.ply)
    [intersect, t, u, v, xcoor] = TriangleRayIntersection (Orig', ray', r(face(:,1),:), r(face(:,2),:), r(face(:,3),:));
    % get the intersection
    as = find(intersect == 1);
    if ~isempty(as)
        project = Orig + ray * min(t(as)) ; % get minimum distance of the intersected side (= closest to 
        result =[result ; project'];
    end    
        
end  
% see =[result ;Orig'];
% 
% figure(3)
% scatter3(see(:,1),see(:,2),see(:,3));
% hold on ;
% trisurf(face, r(:,1),r(:,2),r(:,3),'FaceAlpha', 0.5);
% grid on ;
end

