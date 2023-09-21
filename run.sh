#/bin/bash

./bin/cam_pose_estimator live:2 -c yml/out_camera_calibration.yml -s 0.039 -d ARUCO_MIP_36h12 -ref_id 247 -e 1

#live 实时模式
#live:0 为笔记本摄像头 live:2为USB摄像头
#-c代表相机标定文件，-s代表marker的边长，-d代表marker的字典，-ref_id代表参考marker的id（估计的相机位姿以该ID的marker为参考坐标系），-e代表使用的是enclosed_marker
