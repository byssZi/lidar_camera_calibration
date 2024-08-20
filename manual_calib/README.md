## Introduction

This is a project for LiDAR to camera calibration，including automatic calibration and manual calibration.

## Prerequisites

- Cmake
- opencv 2.4
- eigen 3
- PCL 1.9
- Pangolin

## Manual calibration tool

1. Four input files: 

   ```
   Usage: ./run_lidar2camera <image_path> <pcd_path> <intrinsic_json> <extrinsic_json>
   ```
+ **image_path:** image file from the Camera sensor
+ **pcd_path:** PCD file from the Lidar sensor
+ **intrinsic_json:** Camera intrinsic parameter JSON file
+ **extrinsic_json:** JSON file of initial values of extrinsic parameters between sensors
</br>

2. Calibration panel:

   <img src="./images/lidar2camera_panel.png" width="100%" height="100%" alt="lidar2camera_panel" div align=center /><br>

   The calibration window consists of the left control panel for manual calibration and the right point cloud projection image. Users can check whether the points cloud and the image are aligned by clicking the corresponding button in the panel or using Keyboard as input to adjust the extrinsic parameter. When the points cloud and the image are aligned, the calibration ends, click the save button to save the result.  

   | Extrinsic Params | Keyboard_input | Extrinsic Params | Keyboard_input |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   | Intrinsic Params | Keyboard_input | Intrinsic Params | Keyboard_input |
   | :--------------: | :------------: | :--------------: | :------------: |
   |       + fy       |       i        |       - fy       |       k        |
   |       + fx       |       u        |       - fx       |       j        |

   ```Intensity Color```: LiDAR intensity is recorded as the return strength of a laser beam, partly based on the reflectivity of the object struck by the laser pulse. This button can change the display mode to intensity map display mode. This can help to check if the ground lane lines are aligned.

   ```	Overlap Filter```: Eliminate overlap Lidar points within a depth of 0.4m. 

   ``deg step`` ``t step `` ``fxfy scale`` : These three buttons change the adjustment step for every click or keyboard input.

   ``point size``: Adjust the size of Lidar points in the projection image.

   ``Reset``:  Press button to reset all manual adjustment.

   ``Save Image``:  If the this button was pressed, the results (calibrated image,  extrinsic and intrinsic matrix) are stored by default at running directory `~./manual_calib/`:

   ```
   Extrinsic:
   R:
   0.0121008 -0.999863 -0.0112902
   0.0133341 0.0114512 -0.999846
   0.999838 0.0119484 0.0134707
   t: 0.0134153 -0.352602 -0.575013
   
   Intrinsic:
   2120.29 0 949.828
   0 2071.72 576.237
   0 0 1
   ************* json format *************
   Extrinsic:
   [0.0121008,-0.999863,-0.0112902,0.0134153],[0.0133341,0.0114512,-0.999846,-0.352602],[0.999838,0.0119484,0.0134707,-0.575013],[0,0,0,1]
   
   Intrinsic:
   [2120.29,0,949.828],[0,2071.72,576.237],[0,0,1]
   
   Distortion:
   [-0.108145,0.138668,-0.00379757,-0.00484127]
   ```


