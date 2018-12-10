%% 
[r,face] = read_ply('teabox.ply');
  
   v_1 = r(1,:);
   v_2 = r(2,:);
   v_3 = r(3,:);
   v_4 = r(4,:);
   v_5 = r(5,:);
   v_6 = r(6,:);
   v_7 = r(7,:);
   v_8 = r(8,:); 
   
   world_point_1 = [v_1 ; v_2; v_3; v_4; v_5; v_6];
   world_point_2 = [v_1 ; v_2; v_3; v_4; v_5; v_6; v_8];
   world_point_3 = [v_1 ; v_2; v_3; v_4; v_5; v_8];
   world_point_4 = [v_1 ; v_2; v_3; v_4; v_5; v_7; v_8];
   world_point_5 = [v_1 ; v_2; v_3; v_4; v_7; v_8];
   world_point_6 = [v_1 ; v_2; v_3; v_4; v_6; v_7; v_8];
   world_point_7 = [v_1 ; v_2; v_3; v_4; v_6; v_7];
   world_point_8 = [v_1 ; v_2; v_3; v_4; v_5; v_6; v_7];
 face = face + 1 ;
 
intrinst_m = [2960.37845 0 0;
                0  2960.37845 0;
                1841.68855 1235.23369 1];
cameraParams = cameraParameters('IntrinsicMatrix',intrinst_m);

for n= 1:8
    
   images{n} = imread(sprintf('DSC_97%d.JPG',n+42));
end
% the corner position in each image
image_point_1 = 1.0e+03 * [ 2.3115    1.1115;
                            1.3435    1.1315;
                            1.3795    1.0155;
                            2.2475    1.0035;
                            2.2795    1.5875;
                            1.3755    1.6115];

image_point_2 = 1.0e+03 * [ 1.9275    1.2275;
                            1.4035    0.9835;
                            1.6515    0.9235;
                            2.2075    1.1435;
                            1.9155    1.7315;
                            1.4115    1.3995;
                            2.1795    1.6155];
                        
image_point_3 = 1.0e+03 * [ 1.5395    1.1475;
                            1.5875    0.8475;
                            1.8875    0.8475;
                            1.9515    1.1475;
                            1.5515    1.6515;
                            1.9235    1.6555];


                        
                        
image_point_4 = 1.0e+03 * [ 1.4595    1.0955;
                            2.0795    0.8995;
                            2.3155    0.9715;
                            1.7075    1.1995;
                            1.4755    1.5595;
                            2.3035    1.3915;
                            1.7115    1.6755];

image_point_5 =  1.0e+03 * [1.3515    0.9875;
                            2.2515    1.0035;
                            2.2995    1.1275;
                            1.3035    1.1235;
                            2.2675    1.6035;
                            1.3435    1.5995];

image_point_6 = 1.0e+03 * [ 1.5875    0.8715;
                            2.0595    1.0995;
                            1.7435    1.1755;
                            1.3275    0.9355;
                            2.0515    1.5715;
                            1.7595    1.6675;
                            1.3435    1.3355];

image_point_7 = 1.0e+03 * [ 1.9435    0.8995;
                            1.9875    1.1875;
                            1.5995    1.1875;
                            1.6435    0.9035;
                            1.9715    1.6755;
                            1.6155    1.6795];



image_point_8 = 1.0e+03 * [ 2.3035    1.0235;
                            1.6955    1.2315;
                            1.4675    1.1435;
                            2.0515    0.9555;
                            2.2755    1.4435;
                            1.7075    1.7275;
                            1.4635    1.6115];
                        
[worldOrientation_1,worldLocation_1] = estimateWorldCameraPose(image_point_1,world_point_1,cameraParams,'MaxReprojectionError',2);
[worldOrientation_2,worldLocation_2] = estimateWorldCameraPose(image_point_2,world_point_2,cameraParams,'MaxReprojectionError',2);
[worldOrientation_3,worldLocation_3] = estimateWorldCameraPose(image_point_3,world_point_3,cameraParams,'MaxReprojectionError',2);
[worldOrientation_4,worldLocation_4] = estimateWorldCameraPose(image_point_4,world_point_4,cameraParams,'MaxReprojectionError',2);
[worldOrientation_5,worldLocation_5] = estimateWorldCameraPose(image_point_5,world_point_5,cameraParams,'MaxReprojectionError',5);
[worldOrientation_6,worldLocation_6] = estimateWorldCameraPose(image_point_6,world_point_6,cameraParams,'MaxReprojectionError',2);
[worldOrientation_7,worldLocation_7] = estimateWorldCameraPose(image_point_7,world_point_7,cameraParams,'MaxReprojectionError',5);
[worldOrientation_8,worldLocation_8] = estimateWorldCameraPose(image_point_8,world_point_8,cameraParams,'MaxReprojectionError',2);



 figure(1);
 grid on;
 col = [0; 6; 4; 3; 4; 6;0;4];
 patch('Faces',face,'Vertices',r,'FaceVertexCData',col,'FaceColor','interp');
 view(3);
 hold on
 plotCamera('Size',0.01,'Orientation',worldOrientation_1,'Location',worldLocation_1);
 plotCamera('Size',0.01,'Orientation',worldOrientation_2,'Location',worldLocation_2);
 plotCamera('Size',0.01,'Orientation',worldOrientation_3,'Location',worldLocation_3);
 plotCamera('Size',0.01,'Orientation',worldOrientation_4,'Location',worldLocation_4);
 plotCamera('Size',0.01,'Orientation',worldOrientation_5,'Location',worldLocation_5);
 plotCamera('Size',0.01,'Orientation',worldOrientation_6,'Location',worldLocation_6);
 plotCamera('Size',0.01,'Orientation',worldOrientation_7,'Location',worldLocation_7);
 plotCamera('Size',0.01,'Orientation',worldOrientation_8,'Location',worldLocation_8);

% use the camera pose to calculate rotationMatrix and translationVector
[rotationMatrix_1,translationVector_1] = cameraPoseToExtrinsics(worldOrientation_1,worldLocation_1);
[rotationMatrix_2,translationVector_2] = cameraPoseToExtrinsics(worldOrientation_2,worldLocation_2);
[rotationMatrix_3,translationVector_3] = cameraPoseToExtrinsics(worldOrientation_3,worldLocation_3);
[rotationMatrix_4,translationVector_4] = cameraPoseToExtrinsics(worldOrientation_4,worldLocation_4);
[rotationMatrix_5,translationVector_5] = cameraPoseToExtrinsics(worldOrientation_5,worldLocation_5);
[rotationMatrix_6,translationVector_6] = cameraPoseToExtrinsics(worldOrientation_6,worldLocation_6);
[rotationMatrix_7,translationVector_7] = cameraPoseToExtrinsics(worldOrientation_7,worldLocation_7);
[rotationMatrix_8,translationVector_8] = cameraPoseToExtrinsics(worldOrientation_8,worldLocation_8);

% implement the function of  2D features back-project onto the 3D model
[result_1, D_1] = image2world_ex2(r,face, images{1}, rotationMatrix_1,translationVector_1 , cameraParams)  ;   
[result_2, D_2] = image2world_ex2(r,face, images{2}, rotationMatrix_2,translationVector_2 , cameraParams)  ;
[result_3, D_3] = image2world_ex2(r,face, images{3}, rotationMatrix_3,translationVector_3 , cameraParams)  ;
[result_4, D_4] = image2world_ex2(r,face, images{4}, rotationMatrix_4,translationVector_4 , cameraParams)  ;
[result_5, D_5] = image2world_ex2(r,face, images{5}, rotationMatrix_5,translationVector_5 , cameraParams)  ;
[result_6, D_6] = image2world_ex2(r,face, images{6}, rotationMatrix_6,translationVector_6 , cameraParams)  ;
[result_7, D_7] = image2world_ex2(r,face, images{7}, rotationMatrix_7,translationVector_7 , cameraParams)  ;
[result_8, D_8] = image2world_ex2(r,face, images{8}, rotationMatrix_8,translationVector_8 , cameraParams)  ;
result = [result_1;
          result_2;
          result_3;
          result_4;
          result_5;
          result_6;
          result_7;
          result_8];
D = [D_1 D_2 D_3 D_4 D_5 D_6 D_7 D_8];      
figure(2);   
scatter3(result(:,1),result(:,2),result(:,3),10);
%%
result = load('result.mat');
D = load('feature.mat');
D = D.D ;
result = result.result ;
images = cell(51,74);

LoopOverAll = 0; % False
if ~LoopOverAll
    n = 54
    P = fullfile("exercise02\data\images\detection",sprintf('DSC_97%d.JPG',n))
    images{n} = imread(P);
    X = rgb2gray(images{n}) ; % convert to grayscale
    [fb,db] = vl_sift(single(X));
    [matches, scores] = vl_ubcmatch(D, db, 3.6);
    image_point = fb(1:2,matches(2,:))';
    world_point = result(matches(1,:),:);

    Max = 0; 
    t = 2; 
    N = 800 ;
    pnp_err = 0 ;
    %Ransac
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
            Ransac_worldOrientation = worldOrientation;
            Ransac_worldLocation = worldLocation;
            Ransac_P = P ;
            inlier = image_point(e,:);
        end
    end
    line_point = [ r ones([size(r,1),1])] * Ransac_P;
    line_point(:,1) = line_point(:,1)./line_point(:,3);
    line_point(:,2) = line_point(:,2)./line_point(:,3);
    line_point(:,3) = [] ;
    figure(n);
    imshow(X);
    hold on;
    plot(image_point(:,1),image_point(:,2),'r*','Color','r');
    plot(inlier(:,1),inlier(:,2),'r*','Color','g');

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

if LoopOverAll
    for n= 51:74 % loop over all the given training images
        P=fullfile("exercise02\data\images\detection",sprintf('DSC_97%d.JPG',n))
        images{n} = imread(P);
        X = rgb2gray(images{n}) ; % convert to grayscale
        [fb,db] = vl_sift(single(X));
        [matches, scores] = vl_ubcmatch(D, db, 3.6);

        image_point = fb(1:2,matches(2,:))';
        world_point = result(matches(1,:),:);

        Max = 0; 
        t = 3; 
        N = 800 ;
        pnp_err = 0 ;
    % i = 5 ;
    % subplot(1,2,1);
    % imshow(X);
    % hold on;
    % plot(image_point(:,1),image_point(:,2),'r*','Color','r');
    % plot(image_point(i,1),image_point(i,2),'r*','Color','g');
    % subplot(1,2,2);
    % scatter3(world_point(:,1),world_point(:,2),world_point(:,3),50);


        %Ransac
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
                Ransac_worldOrientation = worldOrientation;
                Ransac_worldLocation = worldLocation;
                Ransac_P = P ;
                inlier = image_point(e,:);
            end
        end
        line_point = [ r ones([size(r,1),1])] * Ransac_P;
        line_point(:,1) = line_point(:,1)./line_point(:,3);
        line_point(:,2) = line_point(:,2)./line_point(:,3);
        line_point(:,3) = [] ;
        figure(n);
        imshow(X);
        hold on;
        plot(image_point(:,1),image_point(:,2),'r*','Color','r');
        plot(inlier(:,1),inlier(:,2),'r*','Color','g');

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

        % Saving the images
        fullFileName = fullfile('C:\Users\Lenny\Documents\Studium_Robotics (M.Sc.)\Semester 1\Tracking and Detection in CV\Exercise 2\OutputImages', sprintf('DSC_97%d.jpg',n))
        saveas(figure(n), fullFileName)
        %imwrite(X, fullFileName);    
    end
end
 
%  figure(7);
%  grid on;
%  col = [0; 6; 4; 3; 4; 6;0;4];
%  patch('Faces',face,'Vertices',r,'FaceVertexCData',col,'FaceColor','interp');
%  view(3);
%  hold on
%  plotCamera('Size',0.01,'Orientation',Ransac_worldOrientation,'Location',Ransac_worldLocation);
