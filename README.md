# Lidar Camera Calibration
***
一种手-自动相结合的lidar-camera标定工具，适配了ros1和ros2  
自动标定部分参考项目 https://github.com/xmba15/automatic_lidar_camera_calibration  
手动标定部分参考项目 https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2camera/manual_calib
***
# 环境准备
```bash
rapidjson
pangolin
```
***
# 标定前准备
1.将相机内参填入`data/samples/camera_info.json`  
2.将初始大概相关位置填入`data/samples/initial_guess.json`  
3.修改`data/samples/calibration_handler_param.json`中的`path_to_initial_guess`，`path_to_images`，`path_to_point_clouds`，`path_to_camera_info`的路径为自己的路径  
4.修改camera，lidar话题名称，ros1为`ros1/sensors_calib_ros1.cpp`，ros2为`ros2/sensors_calib_ros2.cpp`  
5.开启camera，lidar驱动进行标定(lidar点云必须为x,y,z,intensity字段) 
***
# ros1下自动标定 
## Step1
```bash
# build in ros1
make appsros1
```
## Step2
```bash
# run in ros1
./build/devel/lib/sensors_calib/sensors_calib_ros1 ./data/samples/calibration_handler_param.json
```
按p键后按回车键截取点云和图像(尽量选取6～8组点云反射强度差距要大的不同场景数据)，按e键后按回车开始自动标定
## Step3
生成结果保存在`data/result`文件夹下
***
# ros2下自动标定
## Step1
```bash
# build in ros2
make appsros2
```
## Step2
```bash
# run in ros2
./build/ros2/sensors_calib_ros2 ./data/samples/calibration_handler_param.json
```
按p键后按回车键截取点云和图像(尽量选取6～8组点云反射强度差距要大的不同场景数据)，按e键后按回车开始自动标定
## Step3
生成结果保存在`data/result`文件夹下
***
# 手动标定
## Step1 
1.将相机内参填入`manual_calib/data/center_camera-intrinsic.json`文件(如果标定选取用的图片来自自动标定截取保存的图片，则不需要填入畸变系数k1,k2,p1,p2,k3，因为自动标定截取保存的图片已经去过畸变)  
2.将初始外参填入`manual_calib/data/top_center_lidar-to-center_camera-extrinsic.json`文件(可以填入自动标定的结果，用于手动微调)
## Step2
```bash
# run in manual calib
./build/manual_calib/run_lidar2camera <img_path> <pcd_path> ./manual_calib/data/center_camera-intrinsic.json ./manual_calib/data/top_center_lidar-to-center_camera-extrinsic.json
```
## Step3
生成结果保存在`manual_calib/data`文件夹下
***

