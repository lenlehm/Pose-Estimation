function [RANSAC_Orientation, RANSAC_Location, inlier, Ransac_P] = RANSAC(image, N, t)
    Max = 0;
    intrinst_m = [2960.37845 0 0;
                    0  2960.37845 0;
                    1841.68855 1235.23369 1];
    cameraParams = cameraParameters('IntrinsicMatrix',intrinst_m);

    % get the previously stored image information
    result = load('result.mat');
    D = load('feature.mat');
    D = D.D ;
    result = result.result ;
    X = rgb2gray(image) ; % convert to grayscale
    [fb, db] = vl_sift(single(X));
    [matches, scores] = vl_ubcmatch(D, db); %, 3.6

    image_point = fb(1:2,matches(2,:))';
    world_point = result(matches(1,:),:);
    pnp_err = 0;
    
    for i = 1:N
        perm = randperm(size(matches,2)) ;
        image_point_random =image_point( perm(1:4),:);

        world_point_random = world_point(perm(1:4),:) ;         

        try 
            [Orientation,Location,status] = estimateWorldCameraPose(image_point_random,world_point_random,cameraParams);
        % If PnP doesn's find the solution, change the point
        catch ME
            pnp_err = pnp_err +1 ;
            continue;
       end
        [rotationMatrix,translationVector] = cameraPoseToExtrinsics(Orientation,Location);
         P = cameraMatrix(cameraParams,rotationMatrix,translationVector);

        estimate_point = [world_point ones([size(matches,2),1])] * P;
        estimate_point(:,1) = estimate_point(:,1)./estimate_point(:,3);
        estimate_point(:,2) = estimate_point(:,2)./estimate_point(:,3);
        estimate_point(:,3) = [] ;
        err = estimate_point - image_point;
        [distance, idx] = pdist2(estimate_point, image_point, 'euclidean', 'Smallest',1);
        e = find(distance(1, idx) < t);
        %err = err(:,1).^2+err(:,2).^2;
        %e = find(err.^0.5 < t);
        if size(e,1) > Max
            Max = size(e,1);
            RANSAC_Orientation = Orientation;
            RANSAC_Location = Location;
            Ransac_P = P ;
            inlier = image_point(e,:);
        end
    end
end

