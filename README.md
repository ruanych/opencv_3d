# OpenCV 3D

## Brief

This repository implements some algorithms related to point clouds and is developed based on [OpenCV](https://github.com/opencv/opencv).


## Quick Start

```shell
git clone https://github.com/Ryyyc/opencv_3d.git
mkdir opencv_3d/build && cd opencv_3d/build
cmake ..
make
./opencv_3d ../samples/Cassette_GT_.ply-sampled-0.2.ply
```

Please make sure to have an OpenCV environment, or see [OpenCV installation tutorial](https://docs.opencv.org/4.5.1/df/d65/tutorial_table_of_content_introduction.html).

If [cmake](https://cmake.org/) cannot automatically detect the OpenCV installation path, please set `OpenCV_DIR` before `find_package(OpenCV REQUIRED)` in [CMakeLists.txt](./CMakeLists.txt) file.

```cmake
set(OpenCV_DIR ".../opencv/build/x64/vc15/lib")
```

## OpenCV and Point Cloud

#### Point cloud representation

Use data structure `cv::Mat`(size: n x 3) to store point cloud 3D coordinate information


## Point Cloud Related Algorithms


### Voxel Grid Filter Sampling

Creates a 3D voxel grid (a set of tiny 3D boxes in space) over the input point cloud data, in each voxel (i.e., 3D box), all the points present will be approximated (i.e., downsampled) with the point closest to their centroid.


### Total Least Squares Plane Estimate

Fit the plane by the Total Least Squares.

More in [Total-Least-Squares-in-3D.pdf](Total-Least-Squares-in-3D.pdf)
