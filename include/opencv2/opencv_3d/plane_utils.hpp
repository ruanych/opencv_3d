#ifndef OPENCV_OPENCV_3D_PLANE_UTILS_HPP
#define OPENCV_OPENCV_3D_PLANE_UTILS_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace cv {
    namespace _3d {

/**
 * @brief Fit the plane by the Total Least Squares
 *
 * @param input_pts Point cloud, three-dimensional points
 * @param inlier_idxs Plane inlier index in input_pts
 * @param plane_coeffs Plane coefficient  ax + by + cz + d = 0 (output)
 * @return True if the fitting result is valid, otherwise false
 */
bool totalLeastSquaresPlaneEstimate(const cv::Mat &input_pts, std::vector<int> &inlier_idxs,
                                    cv::Vec4f &plane_coeffs);

/**
* @brief Check whether the plane normal vector is the same as the given normal vector
*
* @param actual_plane Plane
* @param expect_normal Given normal vector, [0, 0, 0] represents any normal vector
* @param thr Error threshold ( 1 - (cos<plane_normal, given_normal>)^2 < thr )
* @return True if error is less than the threshold, otherwise false
*/
bool checkPlaneNormalSame(cv::Vec4f &actual_plane, cv::Vec3f &expect_normal, double thr);

/**
 * @brief Check if the two planes are the same plane
 *
 * @param p1 Plane 1  a1x + b1y + c1z + d1 = 0
 * @param p2 Plane 2  a1x + b1y + c1z + d1 = 0
 * @param thr Plane error threshold
 * @return True if error less threshold, otherwise false
 */
bool checkPlaneSame(cv::Vec4f &p1, cv::Vec4f &p2, double thr);

/**
 * @brief Get plane inliers
 *
 * @param plane_coeffs Plane coefficient  ax + by + cz + d = 0
 * @param input_pts Point cloud data
 * @param thr The point is plane inlier if the distance from the point to the plane is less than the threshold
 * @param inliers  mark inliers, corresponds to the index of input_pts, true represents is inlier
 * @return Number of inliers
 */
int getPlaneInliers(const cv::Vec4f &plane_coeffs, const cv::Mat &input_pts, float thr,
                    std::vector<bool> &inliers);

/**
 * @brief Get plane inlier indexes
 *
 * @param plane_coeffs Plane coefficient  ax + by + cz + d = 0
 * @param input_pts Point cloud data
 * @param thr The point is plane inlier if the distance from
 *                 the point to the plane is less than the threshold
 * @param inliers  inlier indexes, corresponds to the index of input_pts
 * @return Number of inliers
 */
int getPlaneInlierIdxs(const cv::Vec4f &plane_coeffs, const cv::Mat &input_pts, float thr,
                       std::vector<int> &inliers);
    } // _3d::
} // cv::

#endif // OPENCV_OPENCV_3D_PLANE_UTILS_HPP
