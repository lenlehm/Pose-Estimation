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

for n= 1:8
   images{n} = imread(sprintf('DSC_97%d.JPG',n+42));
end
[worldOrientation_1,worldLocation_1] = estimateWorldCameraPose(image_point_2,world_point_2,cameraParams,'MaxReprojectionError',2);
[rotationMatrix,translationVector] = cameraPoseToExtrinsics(worldOrientation_1,worldLocation_1);
a = cameraMatrix(cameraParams,rotationMatrix,translationVector);
 P = a';
 Q = P(:,1:3); q = P(:,4);
 Orig = -inv(Q) * q ;


I = rgb2gray(images{n}) ;
%imshow(I);
%hold on ;

peak_thresh = 0;
[f,d] = vl_sift(single(I));
disp(f)
h1 = vl_plotframe(f) ;
set(h1,'color','b','linewidth',3) ;
% [x,y]= getpts;
lamda = 1 ;
result = [] ;
C = [];
Ray = [] ;
for i=1:size(f,2)
    ray = inv(lamda *  Q) * [f(1:2,i)' 1]';
    ray = ray / norm(ray);
    [intersect, t, u, v, xcoor] = TriangleRayIntersection (Orig', ray', r(face(:,1),:), r(face(:,2),:), r(face(:,3),:));
    as = find(intersect == 1);
    Ray =[ Ray ; ray'];
    if ~isempty(as)
        C = [C;f(1:2,i)'];
        project = Orig + ray * min(t(as)) ;
        result =[result ; project'];
    end    
        
end
plot(C(:,1),C(:,2),'r*');
see =[result ;Orig'];

figure(3)

scatter3(see(:,1),see(:,2),see(:,3),1);
hold on ;
trisurf(face, r(:,1),r(:,2),r(:,3),'FaceAlpha', 0.5)
