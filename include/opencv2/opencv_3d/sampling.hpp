#ifndef OPENCV_OPENCV_3D_SAMPLING_HPP
#define OPENCV_OPENCV_3D_SAMPLING_HPP

#include <opencv2/core.hpp>

namespace cv {
    namespace _3d {

/**
 * @brief Voxel Grid filter down sampling:
 * Creates a 3D voxel grid (a set of tiny 3D boxes in space) over the input
 * point cloud data, in each voxel (i.e., 3D box), all the points present will be
 * approximated (i.e., downsampled) with the point closest to their centroid.
 *
 * @param input_pts  Original point cloud
 * @param length Grid length
 * @param width  Grid width
 * @param height  Grid height
 * @param sampled_pts  Point cloud after sampling
 */
void voxelGrid(cv::InputArray &input_pts, float length, float width, float height, cv::OutputArray &sampled_pts);

    } // _3d::
} // cv::

#endif //OPENCV_OPENCV_3D_SAMPLING_HPP
