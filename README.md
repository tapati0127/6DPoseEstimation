<!-- ABOUT THE PROJECT -->
## About The Project
This repos is the implementation of our paper: "6D Pose Estimation for Robot in object grasping application" in VCCA 2021.  
Watch our demo: https://www.youtube.com/watch?v=11uhnISUyXQ&ab_channel=D%C5%A9ngHu%E1%BB%B3nh%C4%90%E1%BB%A9c  
It consists of 3 parts:
* Object detection with YOLOv4 + 6D pose estimation with Point Pair Feature (PPF) + pose refinement with Iterative Closest Point (ICP).
* Object grasping: Communicatation with Motoman Motomini Robot, Gripper + Grasping Program.
* GUI for our software, which makes easier for users.

### Built With

* Qt5
* OpenCV
* PCL
* realsense2 
* ViSP

<!-- GETTING STARTED -->
## Getting Started


### Prerequisites
* Download all requirements.
* Note: In OpenCV, please download contrib modules and replace Surface Matching module by our customized one in: https://github.com/tapati0127/surface_matching and rebuild this module again.

