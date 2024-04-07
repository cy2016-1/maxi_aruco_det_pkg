# aruco二维码功能包

环境需要opencv+opencv_contrib  

## 使用方法

相机内参和畸变参数在condig/camera.yaml里填写  
aruco_det.launch里设置aruco字典，要识别的aruco二维码id，aruco二维码的边长，订阅的图像话题名称。  
```
roslaunch aruco_det aruco_det.launch
```

aruco二维码检测得到的平移向量和旋转向量存放在了/aruco/pose话题里发布出来，可以订阅使用。  

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


