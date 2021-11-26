#include "opencv2/opencv_3d/segmentation.hpp"
#include "opencv2/opencv_3d/ptcloud_utils.hpp"
#include "opencv2/opencv_3d/sampling.hpp"

namespace cv {
namespace _3d {

inline void
localOptimization(const cv::Mat &input_pts, cv::Vec4f &plane_coeffs, std::vector<int> &inlier_idxs,
                  float thr, int max_lo_iters, int max_lo_inliers, cv::Vec3f *normal, double normal_diff_thr)
{
    int best_inls = static_cast<int>(inlier_idxs.size()), num_inls = best_inls;
    if (best_inls < max_lo_inliers) return;
    cv::Vec4f plane_coeffs_new;
    std::vector<int> inlier_sample(max_lo_inliers);

    for (int lo_iter = 0; lo_iter < max_lo_iters; ++lo_iter)
    {
        // Randomly select some points from inlier_idxs to fit the plane
        cv::randShuffle(inlier_idxs);
        for (int i = 0; i < max_lo_inliers; ++i) inlier_sample[i] = inlier_idxs[i];

        if (!cv::_3d::totalLeastSquaresPlaneEstimate(input_pts, inlier_sample, plane_coeffs_new))
            continue;

        if (normal != nullptr && (!cv::_3d::checkPlaneNormalSame(plane_coeffs_new, *normal, normal_diff_thr)))
            continue;

        num_inls = cv::_3d::getPlaneInlierIdxs(plane_coeffs_new, input_pts, thr, inlier_idxs);

        if (best_inls < num_inls)
        {
            plane_coeffs = plane_coeffs_new;
            best_inls = num_inls;
        }
        else if (best_inls == num_inls)
        {
            break;
        }
    }

    //  let inlier_idxs keep best plane' inlier index (May have been contaminated in the above step)
    if (best_inls > num_inls) cv::_3d::getPlaneInlierIdxs(plane_coeffs, input_pts, thr, inlier_idxs);

} // localOptimization()

int ransacFitPlane(const cv::Mat &input_pts, float thr, int max_iterations, cv::Vec4f &plane_coeffs,
                   cv::Vec3f *normal, double normal_diff_thr)
{
    const int pts_size = input_pts.rows, min_sample_size = 3, max_lo_inliers = 20, max_lo_iters = 10;
    if (pts_size < min_sample_size) return 0;

    cv::RNG rng;
    std::vector<int> min_sample(min_sample_size);
    std::vector<int> inlier_sample(max_lo_inliers);

    std::vector<int> inlier_idxs;
    cv::Vec4f plane_coeffs_;
    int best_inls = 0, num_inls, max_iter = max_iterations;
    for (int iter = 0; iter < max_iter; ++iter)
    {
        // Randomly select some points from the point cloud to fit the plane
        for (int i = 0; i < min_sample_size; ++i) min_sample[i] = rng.uniform(0, pts_size);

        if (!cv::_3d::totalLeastSquaresPlaneEstimate(input_pts, min_sample, plane_coeffs_)) continue;

        if (normal != nullptr && (!cv::_3d::checkPlaneNormalSame(plane_coeffs_, *normal, normal_diff_thr)))
            continue;

        num_inls = cv::_3d::getPlaneInlierIdxs(plane_coeffs_, input_pts, thr, inlier_idxs);

        if (num_inls > best_inls)
        {
            // Local Optimization
            localOptimization(input_pts, plane_coeffs_, inlier_idxs, thr, max_lo_iters,
                              max_lo_inliers, normal, normal_diff_thr);

            // Preserve the best plane so far
            plane_coeffs = plane_coeffs_;
            best_inls = static_cast<int>(inlier_idxs.size());

            const double iter_lower =
                    3 * log(1 - 0.96) / log(1 - pow(float(best_inls) / pts_size, min_sample_size));
            if (!std::isinf(iter_lower) && iter_lower < max_iter)
            {
                max_iter = static_cast<int>(iter_lower);
            }
        }
    }

    return best_inls;
} // ransacFitPlane()

void ransacFitPlanes(cv::InputArray &input_pts, float thr, int max_iterations,
                     std::vector<cv::Vec4f> &planes_coeffs, cv::Mat &labels,
                     int desired_num_planes, float grid_size,
                     cv::Vec3f *normal, double normal_diff_thr)
{
    CV_CheckGT(thr, 0.0f, "The threshold parameter must be greater than 0.");
    CV_CheckGT(max_iterations, 0, "The max iterations parameter must be greater than 0.");
    CV_CheckGT(desired_num_planes, 0, "The number of desired planes_coeffs parameter must be greater than 0.");

    // Get input point cloud data
    cv::_InputArray::KindFlag kind = input_pts.kind();

    CV_CheckType(kind, kind == _InputArray::STD_VECTOR || kind == _InputArray::MAT,
                 "Point cloud data storage type should be vector of Point3 or Mat of size Nx3");

    cv::Mat pts = input_pts.getMat();
    if (kind == _InputArray::STD_VECTOR){
        pts = cv::Mat(static_cast<int>(pts.total()), 3, CV_32F, pts.data);
    } else {
        if (pts.channels() != 1)
            pts = pts.reshape(1, static_cast<int>(pts.total())); // Convert to single channel
        if (pts.cols != 3 && pts.rows == 3)
            cv::transpose(pts, pts);

        CV_CheckEQ(pts.cols, 3, "Invalid dimension of point cloud");
        if (pts.type() != CV_32F)
            pts.convertTo(pts, CV_32F);// Use float to store data
    }

    // Plane fitted using Voxel Grid Filter Sampling point cloud
    std::vector<cv::Vec4f> planes_;
    {
        cv::Mat pts_plane_fit; // Point cloud used to find a plane every time
        if (grid_size > 0)
            cv::_3d::voxelGridSampling(pts_plane_fit, pts, grid_size, grid_size, grid_size);
        else
            pts_plane_fit = pts;

        std::vector<bool> inliers_;
        cv::Vec4f plane_coeffs_;
        int inliers_num = ransacFitPlane(pts_plane_fit, thr, max_iterations, plane_coeffs_,
                                         normal, normal_diff_thr);
        if (inliers_num > 0)
        {
            planes_.emplace_back(plane_coeffs_);
            for (int num_planes = 1; num_planes < desired_num_planes; ++num_planes)
            {
                cv::_3d::getPlaneInliers(plane_coeffs_, pts_plane_fit, thr, inliers_);

                const int pts_fit_size = pts_plane_fit.rows;
                cv::Mat tmp_pts(pts_plane_fit);
                pts_plane_fit = cv::Mat(pts_fit_size - inliers_num, 3, CV_32F);

                float *const tmp_pts_ptr = (float *) tmp_pts.data;
                float *const pts_fit_ptr = (float *) pts_plane_fit.data;
                for (int i = 0, j = 0; i < pts_fit_size; ++i)
                {
                    if (!inliers_[i])
                    {
                        // If it is not inlier of the known plane,
                        //   add the next iteration to find a new plane
                        float *const tmp_ptr_base = tmp_pts_ptr + 3 * i;
                        float *const pts_fit_ptr_base = pts_fit_ptr + 3 * j;
                        *pts_fit_ptr_base = *tmp_ptr_base;
                        *(pts_fit_ptr_base + 1) = *(tmp_ptr_base + 1);
                        *(pts_fit_ptr_base + 2) = *(tmp_ptr_base + 2);
                        ++j;
                    }
                }

                inliers_num = ransacFitPlane(pts_plane_fit, thr, max_iterations, plane_coeffs_,
                                             normal, normal_diff_thr);
                if (inliers_num == 0) break;

                planes_.emplace_back(plane_coeffs_);
            }
        }
    }


    int planes_cnt = (int) planes_.size();
    int pts_size = pts.rows;
    labels = cv::Mat::zeros(pts_size, 1, CV_32S);

    if (planes_cnt > 0)
    {
        //  According to the obtained plane equation,
        //   perform local optimization on the origin cloud data and label it
        int max_lo_inliers = 300, max_lo_iters = 3;

        // Keep the index array of the point corresponding to the original point
        int *orig_pts_idx = new int[pts_size];
        for (int i = 0; i < pts_size; ++i) orig_pts_idx[i] = i;

        int *const labels_ptr = (int *) labels.data;
        std::vector<bool> inliers;
        std::vector<int> inlier_idxs;

        for (int plane_num = 1; plane_num <= planes_cnt; ++plane_num)
        {
            cv::Vec4f plane_coeffs = planes_[plane_num - 1];
            cv::_3d::getPlaneInlierIdxs(plane_coeffs, pts, thr, inlier_idxs);

            // Local Optimization
            localOptimization(pts, plane_coeffs, inlier_idxs, thr, max_lo_iters,
                              max_lo_inliers, normal, normal_diff_thr);
            cv::_3d::getPlaneInliers(plane_coeffs, pts, thr, inliers);
            planes_coeffs.push_back(plane_coeffs);

            pts_size = pts.rows;
            if (plane_num != planes_cnt)
            {
                int best_inls = static_cast<int>(inlier_idxs.size());
                cv::Mat tmp_pts(pts);
                pts = cv::Mat(pts_size - best_inls, 3, CV_32F);

                float *const tmp_pts_ptr = (float *) tmp_pts.data;
                float *const pts_ptr = (float *) pts.data;
                for (int j = 0, i = 0; i < pts_size; ++i)
                {
                    if (!inliers[i])
                    {
                        // If it is not inlier of the known plane,
                        //   add the next iteration to find a new plane
                        orig_pts_idx[j] = orig_pts_idx[i];
                        float *const tmp_ptr_base = tmp_pts_ptr + 3 * i;
                        float *const pts_fit_ptr_base = pts_ptr + 3 * j;
                        *pts_fit_ptr_base = *tmp_ptr_base;
                        *(pts_fit_ptr_base + 1) = *(tmp_ptr_base + 1);
                        *(pts_fit_ptr_base + 2) = *(tmp_ptr_base + 2);
                        ++j;
                    }
                    else
                    {
                        // Otherwise mark a label on this point
                        labels_ptr[orig_pts_idx[i]] = plane_num;
                    }
                }
            }
            else
            {
                for (int i = 0; i < pts_size; ++i)
                {
                    if (inliers[i])
                        labels_ptr[orig_pts_idx[i]] = plane_num;
                }
            }
        }

        delete[] orig_pts_idx;
    }
} // ransacFitPlanes()

} // _3d::
}  // cv::