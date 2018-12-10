function matches = getSIFTfeatures(image1, image2)
%returns the matches of the SIFT keypoints in both images
% load results to obtain world_points from matched keypoints
result = load('result.mat');
result = result.result;

% convert images to grayscale
X1 = rgb2gray(image1);
X2 = rgb2gray(image2);
% set up SIFT descriptor for both images and get matches
peak_thresh = 2;
[f1,d1] = vl_sift(single(X1),'PeakThresh', peak_thresh);
[f2,d2] = vl_sift(single(X2),'PeakThresh', peak_thresh);
[matches, scores] = vl_ubcmatch(d1, d2, 3.6) ;

world_point = result(matches(1,:),:);
image_point = fb(1:2,matches(2,:))';