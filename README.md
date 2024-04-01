# aruco二维码功能包

环境需要opencv+opencv_contrib  

## 使用方法

相机内参和畸变参数在condig/camera.yaml里填写  
aruco_det.launch里设置aruco字典，要识别的aruco二维码id，aruco二维码的边长。  
```
roslaunch aruco_det aruco_det.launch
```

## aruco detection without ros

need opencv3.3.1 and opencv_contrib3.3.1

compile command
```
g++ -o aruco_detection aruco_detection.cpp `pkg-config --cflags --libs opencv`

```
usage
```
./aruco_detection
```


