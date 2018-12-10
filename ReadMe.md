# Pose Estimation in Matlab

In this repository, I calculated Pose Estimations of the camera, as well of the teabox object in the picture. 

The first exercise covers the various Camera Poses and displays them on the image. So where the camera is located in the world frame.

Format: ![Camera Poses](Exercise/1_MATLAB_PoseEstimation/Presentation/CameraPose.png)

The Second Exercise covers the SIFT keypoints to infer the object pose to various other images using PnP and RANSAC
Format: ![Object Poses](Exercise/1_MATLAB_PoseEstimation/Presentation/CameraPose.png)

The thrid exercise dealt with the combination of the 2 previous exercises and displays the camera trajectory on all the images. Therefore the first image was used to estimate its pose along with the world coordinates. The consecutive frames were matched with the SIFT keypoints to infer the camera pose on each frame.
Format: ![Camera Trajectory](Exercise/1_MATLAB_PoseEstimation/Presentation/CameraPose.png)