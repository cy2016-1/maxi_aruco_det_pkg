# aruco二维码功能包

环境需要opencv+opencv_contrib  

## 使用方法

```
rosrun aruco_det aruco_det
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


