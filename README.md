# ProjectLiDARToImage
A tool to project LiDAR point cloud to image, facilitating to evaluate the calibration results(intrinsic parameters of the camera and the extrinsic parameters between the camera and the LiDAR). 

## Dependency
- OpenCV
- PCL

## Compile
mkdir build
cd build
cmake ..
make -j

## Usage
usage: ./ProjectLiDARToImage imgPathName pcdPathName calib_cam_to_cam.txt calib_cam_to_velo.txt outPathName
eg:  ./ProjectLiDARToImage ../example/001.png ../example/001.pcd ..example/calib_cam_to_cam.txt ..example/calib_cam_to_velo.txt ..example/Project_001.png

## Notice
- "calib_cam_to_cam.txt" and "calib_cam_to_velo.txt" should be given in KITTI format.
- Ensure that you have clearly known the difference between "calib_cam_to_velo" and "calib_velo_to_cam". That is, if you use "calib_velo_to_cam", the (R, T) should be changed into (R.inv(), -R.inv() * T).


