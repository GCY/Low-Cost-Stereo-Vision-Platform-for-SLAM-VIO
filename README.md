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

## Reference
 - [ESP32 CAM MJPEG Stream Decoder and Control Library](https://github.com/GCY/ESP32-CAM-MJPEG-Stream-Decoder-and-Control-Library)



LICENSE
-------

MIT License

Copyright (c) 2020 Tony Guo https://github.com/GCY

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
