#include "opencv2/opencv_3d/plane_utils.hpp"

namespace cv {
namespace _3d {

bool totalLeastSquaresPlaneEstimate(const cv::Mat &input_pts, std::vector<int> &inlier_idxs,
                                    cv::Vec4f &plane_coeffs)
{
    const int inlier_cnt = static_cast<int>(inlier_idxs.size());
    CV_CheckGT(inlier_cnt, 2, "The number of plane inlier must be greater than 2");
    const float *pts_ptr = (float *) input_pts.data;

    // Judging the three points collinear
    if (3 == inlier_cnt)
    {
        int id1 = 3 * inlier_idxs[0], id2 = 3 * inlier_idxs[1], id3 = 3 * inlier_idxs[2];
        float x1 = pts_ptr[id1], y1 = pts_ptr[id1 + 1], z1 = pts_ptr[id1 + 2];
        float x2 = pts_ptr[id2], y2 = pts_ptr[id2 + 1], z2 = pts_ptr[id2 + 2];
        float x3 = pts_ptr[id3], y3 = pts_ptr[id3 + 1], z3 = pts_ptr[id3 + 2];
        cv::Vec3f ba(x1 - x2, y1 - y2, z1 - z2);
        cv::Vec3f ca(x1 - x3, y1 - y3, z1 - z3);
        float ba_dot_ca = fabs(ca.dot(ba));
        if (fabs(ba_dot_ca * ba_dot_ca - ba.dot(ba) * ca.dot(ca)) < 0.0001)
        {
            return false;
        }
    }
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < inlier_cnt; ++i)
    {
        const float *pts_ptr_base = pts_ptr + 3 * inlier_idxs[i];
        sum_x += *pts_ptr_base;
        sum_y += *(pts_ptr_base + 1);
        sum_z += *(pts_ptr_base + 2);
    }

    const float mean_x = sum_x / inlier_cnt, mean_y = sum_y / inlier_cnt, mean_z = sum_z / inlier_cnt;

    cv::Mat pd_mat;
    {
        cv::Mat U(inlier_cnt, 3, CV_32F);
        float *U_ptr = (float *) U.data;
        for (int i = 0; i < inlier_cnt; ++i)
        {
            float *ptr_base = (U_ptr + 3 * i);
            const float *pts_ptr_base = pts_ptr + 3 * inlier_idxs[i];
            *ptr_base = *(pts_ptr_base) - mean_x;
            *(ptr_base + 1) = *(pts_ptr_base + 1) - mean_y;
            *(ptr_base + 2) = *(pts_ptr_base + 2) - mean_z;
        }

        pd_mat = U.t() * U;
    }

    cv::Mat eigenvalues;
    cv::Mat eigenvectors(3, 3, CV_32F);
    cv::eigen(pd_mat, eigenvalues, eigenvectors);
    const float *eig_ptr = (float *) eigenvectors.data;

    float a = eig_ptr[6], b = eig_ptr[7], c = eig_ptr[8];
    if (std::isinf(a) || std::isinf(b) || std::isinf(c) || (a == 0 && b == 0 && c == 0))
    {
        return false;
    }

    plane_coeffs = cv::Vec4f(a, b, c, -a * mean_x - b * mean_y - c * mean_z);
    return true;
} // totalLeastSquaresPlaneEstimate()

bool checkPlaneNormalSame(cv::Vec4f &actual_plane, cv::Vec3f &expect_normal, double thr)
{
    CV_CheckGT(thr, 0.0, "The threshold parameter must be greater than 0.");
    double dot_ab = (actual_plane[0] * expect_normal[0] + actual_plane[1] * expect_normal[1] +
                     actual_plane[2] * expect_normal[2]);
    double modulo_square_a = (actual_plane[0] * actual_plane[0] + actual_plane[1] * actual_plane[1] +
                              actual_plane[2] * actual_plane[2]);
    double modulo_square_b = (expect_normal[0] * expect_normal[0] + expect_normal[1] * expect_normal[1] +
                              expect_normal[2] * expect_normal[2]);
    double dot_square_ab = dot_ab * dot_ab;
    double modulo_square_ab = (modulo_square_a * modulo_square_b);
    return (modulo_square_ab - dot_square_ab) * (modulo_square_ab + dot_square_ab) <= thr * modulo_square_ab;
}

bool checkPlaneSame(cv::Vec4f &p1, cv::Vec4f &p2, double thr)
{
    CV_CheckGT(thr, 0.0, "The threshold parameter must be greater than 0.");
    double hom1 = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2] + p1[3] * p1[3]);
    double hom2 = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2] + p2[3] * p2[3]);
    double p1a = p1[0] / hom1, p1b = p1[1] / hom1, p1c = p1[2] / hom1, p1d = p1[3] / hom1;
    double p2a = p2[0] / hom2, p2b = p2[1] / hom2, p2c = p2[2] / hom2, p2d = p2[3] / hom2;
    return (p1a - p2a) * (p1a - p2a) + (p1b - p2b) * (p1b - p2b) + (p1c - p2c) * (p1c - p2c) +
           (p1d - p2d) * (p1d - p2d)
           < thr; // 0.0000001
} // checkPlaneNormalSame()

int getPlaneInliers(const cv::Vec4f &plane_coeffs, const cv::Mat &input_pts, float thr,
                    std::vector<bool> &inliers)
{
    CV_CheckGT(thr, 0.0f, "The threshold parameter must be greater than 0.");

    const int pts_size = input_pts.rows;
    float *const pts_ptr = (float *) input_pts.data;
    float a = plane_coeffs(0), b = plane_coeffs(1), c = plane_coeffs(2), d = plane_coeffs(3);
    float hom = sqrt(a * a + b * b + c * c);
    a = a / hom, b = b / hom, c = c / hom, d = d / hom;
    inliers.resize(pts_size);

    int num_inliers = 0;
//            parallel_for_(Range(0, pts_size), [&](const cv::Range &range) {
//                for (int i = range.start; i < range.end; ++i) {
//                    float *const pts_ptr_base = pts_ptr + 3 * i;
//                    float dist = a * (*(pts_ptr_base)) + b * (*(pts_ptr_base + 1)) + c * (*(pts_ptr_base + 2)) + d;
//                    if (dist < thr && dist > -thr) {
//                        inliers[i] = true;
//                        ++num_inliers;
//                    } else {
//                        inliers[i] = false;
//                    }
//                }
//            });

    for (int i = 0; i < pts_size; ++i)
    {
        float *const pts_ptr_base = pts_ptr + 3 * i;
        float dist = a * (*(pts_ptr_base)) + b * (*(pts_ptr_base + 1)) + c * (*(pts_ptr_base + 2)) + d;
        if (dist < thr && dist > -thr)
        {
            inliers[i] = true;
            ++num_inliers;
        }
        else
        {
            inliers[i] = false;
        }
    }
    return num_inliers;
} // getPlaneInliers()

int getPlaneInlierIdxs(const cv::Vec4f &plane_coeffs, const cv::Mat &input_pts, float thr,
                       std::vector<int> &inliers)
{
    CV_CheckGT(thr, 0.0f, "The threshold parameter must be greater than 0.");

    const int pts_size = input_pts.rows;
    float *const pts_ptr = (float *) input_pts.data;
    float a = plane_coeffs(0), b = plane_coeffs(1), c = plane_coeffs(2), d = plane_coeffs(3);
    float hom = sqrt(a * a + b * b + c * c);
    a = a / hom, b = b / hom, c = c / hom, d = d / hom;
    inliers.clear();

    for (int i = 0; i < pts_size; ++i)
    {
        float *const pts_ptr_base = pts_ptr + 3 * i;
        float dist = a * (*(pts_ptr_base)) + b * (*(pts_ptr_base + 1)) + c * (*(pts_ptr_base + 2)) + d;
        if (dist < thr && dist > -thr)
        {
            inliers.push_back(i);
        }
    }
    return static_cast<int>(inliers.size());
} // getPlaneInlierIdxs()

} // _3d::
} // cv::