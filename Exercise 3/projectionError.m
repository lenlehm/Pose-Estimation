function err = projectionError(projPoints, realPoints)
% goal minimize reprojecection error of keypoints in previous frame
% and current frame.
% returns the error as euclidean distance among the 2 points
err = pdist2(projPoints,realPoints,'euclidean','smallest',1);