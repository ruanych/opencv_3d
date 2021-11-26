#ifndef OPENCV_OPENCV_3D_PTCLOUD_UTILS_HPP
#define OPENCV_OPENCV_3D_PTCLOUD_UTILS_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace cv {
    namespace _3d {

/**
 * @brief Get cv::Mat with Nx3 or 3xN type CV_32FC1 from cv::InputArray
 *
 * @param input_pts Point cloud xyz data
 * @param[out] mat Point cloud xyz data in cv::Mat with Nx3 or 3xN type CV_32FC1
 * @param arrangement_of_points The arrangement of point data in the matrix,
 *                              0 by row (Nx3), 1 by column (3xN)
 * @param clone_data Flag to specify whether data cloning is mandatory
 *
 * @note The following cases will clone data even if flag clone_data is false:
 *       1. Data is discontinuous in memory \n
 *       2. Data type is not float \n
 *       3. The original arrangement of data is not the same as the expected new arrangement. \n
 *          For example, transforming from
 *          Nx3(x1, y1, z1, ..., xn, yn, zn) to 3xN(x1, ..., xn, y1, ..., yn, z1, ..., zn)
 *
 */
inline void _getMatFromInputArray(InputArray input_pts, Mat &mat,
                                  int arrangement_of_points = 0, bool clone_data = false)
{
    CV_Check(input_pts.dims(), input_pts.dims() < 3,
             "Only support data with dimension less than 3.");

    // Guaranteed data can construct N×3 point clouds
    int rows = input_pts.rows(), cols = input_pts.cols(), channels = input_pts.channels();
    size_t total = rows * cols * channels;
    CV_Check(total, total % 3 == 0,
             "total = input_pts.rows() * input_pts.cols() * input_pts.channels() must be an integer multiple of 3");

    /**
     Layout of point cloud data in memory space.
     arrangement 0 : x1, y1, z1, ..., xn, yn, zn
                    For example, the input is std::vector<Point3d>, or std::vector<int>,
                    or cv::Mat with type N×1 CV_32FC3
     arrangement 1 : x1, ..., xn, y1, ..., yn, z1, ..., zn
                    For example, the input is cv::Mat with type 3×N CV_32FC1
     */
    int ori_arrangement = (channels == 1 && rows == 3 && cols != 3) ? 1 : 0;

    // Convert to single channel without copying the data.
    mat = ori_arrangement == 0 ? input_pts.getMat().reshape(1, (int) (total / 3))
                               : input_pts.getMat();

    if (ori_arrangement != arrangement_of_points)
    {
        Mat tmp;
        transpose(mat, tmp);
        swap(mat, tmp);
    }

    if (mat.type() != CV_32F)
    {
        Mat tmp;
        mat.convertTo(tmp, CV_32F); // Use float to store data
        swap(mat, tmp);
    }

    if (clone_data || (!mat.isContinuous()))
    {
        mat = mat.clone();
    }

}

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

#endif // OPENCV_OPENCV_3D_PTCLOUD_UTILS_HPP
