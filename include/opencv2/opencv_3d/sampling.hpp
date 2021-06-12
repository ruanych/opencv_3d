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
 * @param input_pts  Original point cloud, vector of Point3 or Mat of size Nx3
 * @param length Grid length
 * @param width  Grid width
 * @param height  Grid height
 * @param sampled_pts  Point cloud after sampling
 */
void voxelGrid(cv::InputArray &input_pts, float length, float width, float height,
               cv::OutputArray &sampled_pts);

/**
 * @brief Point cloud sampling by randomly select points
 *
 * @param input_pts  Original point cloud, vector of Point3 or Mat of size Nx3
 * @param sampled_pts_size The desired point cloud size after sampling
 * @param sampled_pts  Point cloud after sampling
 */
void randomSampling(cv::InputArray &input_pts, int sampled_pts_size, cv::OutputArray &sampled_pts);

/**
 * @brief Point cloud sampling by randomly select points
 *
 * @param input_pts  Original point cloud, vector of Point3 or Mat of size Nx3
 * @param sampled_scale The percentage of the sampled point cloud to the original size,
 *                      that is, sampled size = original size * sampled_scale, range (0, 1)
 * @param sampled_pts  Point cloud after sampling
 */
void randomSampling(cv::InputArray &input_pts, float sampled_scale, cv::OutputArray &sampled_pts);

/**
 * @brief Farthest Point Sampling:
 * Input point cloud C, sampled point cloud S, S initially has a size of 0
 * 1. Randomly take a seed point from C and put it into S
 * 2. Find a point in C that is the farthest away from S and put it into S
 * The distance from point to S set is the smallest distance from point to all points in S
 *
 * @param input_pts  Original point cloud, vector of Point3 or Mat of size Nx3
 * @param sampled_pts_size The desired point cloud size after sampling
 * @param sampled_pts  Point cloud after sampling
 * @param dist_lower_limit Sampling is terminated early if the distance from
 *                  the farthest point to S is less than dist_lower_limit, default 0
 */
void farthestPointSampling(cv::InputArray &input_pts, int sampled_pts_size,
                           cv::OutputArray &sampled_pts, float dist_lower_limit = 0);

/**
 * @brief Details in farthestPointSampling(cv::InputArray, int, cv::OutputArray, float)
 *
 * @param input_pts  Original point cloud, vector of Point3 or Mat of size Nx3
 * @param sampled_scale The percentage of the sampled point cloud to the original size,
 *                      that is, sampled size = original size * sampled_scale, range (0, 1)
 * @param sampled_pts  Point cloud after sampling
 * @param dist_lower_limit Sampling is terminated early if the distance from
 *                  the farthest point to S is less than dist_lower_limit, default 0
 */
void farthestPointSampling(cv::InputArray &input_pts, float sampled_scale,
                           cv::OutputArray &sampled_pts, float dist_lower_limit = 0);

} // _3d::
} // cv::

#endif //OPENCV_OPENCV_3D_SAMPLING_HPP
