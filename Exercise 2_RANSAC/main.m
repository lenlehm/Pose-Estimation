[r,face] = read_ply('teabox.ply');
result = load('result.mat');
D = load('feature.mat');
D = D.D ;
result = result.result ;

n = 54
P=fullfile("exercise02\data\images\detection",sprintf('DSC_97%d.JPG',n))
images{n} = imread(P);

T = 2; 
N = 800 ;

[RANSAC_Orientation,RANSAC_Location, inlier, Ransac_P] = RANSAC2(images{n}, N, T)
line_point = [ r ones([size(r,1),1])] * Ransac_P;
line_point(:,1) = line_point(:,1)./line_point(:,3);
line_point(:,2) = line_point(:,2)./line_point(:,3);
line_point(:,3) = [] ;
figure(n);
imshow(images{n});
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
%fullFileName = fullfile('C:\Users\Lenny\Documents\Studium_Robotics (M.Sc.)\Semester 1\Tracking and Detection in CV\Exercise 2\OutputImages', sprintf('DSC_97%d.jpg',n))
%saveas(figure(n), fullFileName)
%imwrite(X, fullFileName);    

%  figure(7);
%  grid on;
%  col = [0; 6; 4; 3; 4; 6;0;4];
%  patch('Faces',face,'Vertices',r,'FaceVertexCData',col,'FaceColor','interp');
%  view(3);
%  hold on
%  plotCamera('Size',0.01,'Orientation',Ransac_worldOrientation,'Location',Ransac_worldLocation);
