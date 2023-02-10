# 3DMC-Evaluate-Stereo-Camera

My 2022 summer research project with the Loughborough Intelligent Automation Centre:

"An Automated Approach to Evaluate a Stereo Camera’s Ability to Generate Point Clouds".

We have used a genetic algorithm to optimise the Zed 2 stereo camera's generated point clouds. 
We measured the accuracy of point clouds through comparing flat surfaces (as ground truth) with the point clouds scan of the same surface.
Flatness is then compared using CloudCompare's fit plane method to determine the Root Mean Square Error.

This repository contains all the code used for my oral presentation at the 2022 3D Metrology Conference.

Pre-requisites:
  * Windows PC
  * Stereolabs Zed 2 Camera
  * CloudCompare
  
Dependencies:
  * sys
  * pyzed (Zed Camera API)
  * numpy
  * time 
  * os 
  * glob
  * cv2 (OpenCV)
  * open3d 
  * random
