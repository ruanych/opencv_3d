#ifndef OPENCV_OPENCV_3D_SEGMENTATION_HPP
#define OPENCV_OPENCV_3D_SEGMENTATION_HPP

#include <opencv2/core.hpp>
#include <vector>

namespace cv {
namespace _3d {

/**
 * @brief Fit plane by RANSAC algorithm
 *
 * @param input_pts  Point cloud data
 * @param thr  The point is plane inlier if the distance from
 *                  the point to the plane is less than the threshold
 * @param max_iterations The maximum number of iterations for RANSAC
 * @param plane_coeffs  (Output) Plane equation, ax + by + cz + d = 0
 * @param normal Normal vector constraint, nullptr means no constraint is used, otherwise
 *               the detected plane normal vector satisfies the constraint, default nullptr
 * @param normal_diff_thr Plane normal vector constraint error threshold,
 *                        effective if normal is not nullptr, default 0.06
 */
int ransacFitPlane(const cv::Mat &input_pts, float thr, int max_iterations, cv::Vec4f &plane_coeffs,
                   cv::Vec3f *normal = nullptr, double normal_diff_thr = 0.06);

/**
 * @brief Fit multiple planes
 *
 * @param input_pts  Point cloud data
 * @param thr  The point is plane inlier if the distance from
 *                  the point to the plane is less than the threshold
 * @param max_iterations The maximum number of iterations for RANSAC
 * @param planes_coeffs  (Output) Plane equations, ax + by + cz + d = 0
 * @param labels (Output) 0 means it does not belong to any plane,
 *               otherwise the plane is marked with a different number
 * @param desired_num_planes  Number of desired planes, default 1
 * @param grid_size  Sampling grid size, if not greater than 0, it means no sampling, default -1
 * @param normal Normal vector constraint, nullptr means no constraint is used, otherwise
 *               the detected plane normal vector satisfies the constraint, default nullptr
 * @param normal_diff_thr Plane normal vector constraint error threshold,
 *                        effective if normal is not nullptr, default 0.06
 */
void ransacFitPlanes(cv::InputArray &input_pts, float thr, int max_iterations,
                     std::vector<cv::Vec4f> &planes_coeffs, cv::Mat &labels,
                     int desired_num_planes = 1, float grid_size = -1,
                     cv::Vec3f *normal = nullptr, double normal_diff_thr = 0.06);

} // _3d::
}  // cv::

#endif //OPENCV_OPENCV_3D_SEGMENTATION_HPP
