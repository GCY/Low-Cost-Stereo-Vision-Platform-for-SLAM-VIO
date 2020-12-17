# Low-Cost-Stereo-Vision-Platform-for-SLAM-VIO

![v1.3](https://github.com/GCY/Low-Cost-Stereo-Vision-Platform-for-SLAM-VIO/blob/master/res/v1.3.jpg?raw=true)

![diagram](https://github.com/GCY/Low-Cost-Stereo-Vision-Platform-for-SLAM-VIO/blob/master/res/diagram.png?raw=true)


![demo](https://github.com/GCY/Low-Cost-Stereo-Vision-Platform-for-SLAM-VIO/blob/master/res/demo.gif) 

 - Step 1. Capture the [chessboard pattern](https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png) picture from Left and Right Camera.
 - Step 2. Run "Stereo Camera Calibration"
 - Step 3. Calibration stereo camera phase, execute initUndistortRectifyMap with "extrinsics.yml" and "intrinsics.yml" befor cv::remap
 - Step 4. Create StereoBM or SGBM object to calculate depth map.
 - Step 5. Cet point cloud from depth map and baseline.

## Mac OS X

### Dependence
 - wxWidgets 3.0.4
 - OpenCV 4.4.0
 - libcurl 7.54.0
 
### Build

```cpp

cd ./App/Mac/wxStereoCamera
make

```

## Win10

### Dependence
 - wxWidgets 3.1.4
 - OpenCV 4.4.0
 - libcurl 7.67.0
 
 ### Build
 ```cpp
 Build App/Win10/wxStereoCamera/wxStereoCamera.sln with Visual Studio Community 2019
 ```
