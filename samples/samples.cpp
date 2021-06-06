#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/opencv_3d/sampling.hpp"
#include "opencv2/opencv_3d/plane_utils.hpp"
#include "opencv2/opencv_3d/segmentation.hpp"
#include "samples.hpp"
#include "utils.hpp"

namespace cv {
namespace _3d {

void sampling_sample(const std::string &test_file_path, float grid_size)
{
    std::cout << "=========== SAMPLING ===========\n";
    if (grid_size > 0)
    {
        cv::Mat point_cloud;
        if (read_point_cloud_ply_to_mat(test_file_path, point_cloud, false))
        {
            std::cout << "Read point cloud data successfully, point cloud size: " << point_cloud.size << std::endl;

            cv::Mat sampled_pts;
            clock_t start = clock();
            cv::_3d::voxelGrid(point_cloud, grid_size, grid_size, grid_size, sampled_pts);
            clock_t end = clock();
            std::cout << "After sampled, point cloud size: " << sampled_pts.size << std::endl;
            std::cout << "Sampling time cost: " << (1000.0 * (end - start)) / CLOCKS_PER_SEC << " ms" << std::endl;
            if (save_point_cloud_ply(test_file_path + std::to_string(grid_size) + "-sampled.ply", sampled_pts, true))
            {
                std::cout << "Save sampled point cloud successful, location: "
                          << test_file_path + std::to_string(grid_size) + "-sampled.ply" << std::endl;;
            }
        }
    }
    else
    {
        std::cout << "Invalid grid size length." << std::endl;
    }

    std::cout << "-----------------------------------\n\n";
} // sampling_sample()

void total_least_squares_sample()
{
    std::cout << "=========== TOTAL LEAST SQUARES ===========\n";
    cv::Vec4f ori_coeffs(1.0f / cv::sqrt(3.0f), 1.0f / cv::sqrt(3.0f), 1.0f / cv::sqrt(3.0f), 1);

    float a = ori_coeffs[0], b = ori_coeffs[1], c = ori_coeffs[2], d = ori_coeffs[3]; // ax + by + cz + d = 0
    std::cout << "Equation coefficients used to generate the plane: " << ori_coeffs << std::endl;

    int pts_size = 25000;
    cv::Mat point_cloud(pts_size, 3, CV_32F);
    float *const pts_ptr = (float *) point_cloud.data;

    cv::RNG rng;
    for (int i = 0; i < pts_size; ++i)
    {
        int ii = 3 * i;
        float x = rng.uniform(0.0f, 300000.0f);
        float y = rng.uniform(0.0f, 300000.0f);
        float z = (-d - a * x - b * y) / c;
        pts_ptr[ii] = x;
        pts_ptr[ii + 1] = y;
        pts_ptr[ii + 2] = z;
    }


    cv::Vec4f coeffs;
    std::vector<int> idx = std::vector<int>();
    for (int i = 0; i < pts_size; ++i) idx.push_back(i);

    clock_t start = clock();
    cv::_3d::totalLeastSquaresPlaneEstimate(point_cloud, idx, coeffs);
    clock_t end = clock();
    std::cout << "Point cloud size: " << point_cloud.size << std::endl;
    std::cout << "Fitted plane equation coefficients: " << coeffs << std::endl;
    std::cout << "Plane fitting time cost: " << (1000.0 * (end - start)) / CLOCKS_PER_SEC << " ms" << std::endl;
    std::cout << "-----------------------------------\n\n";
} // total_least_squares_sample()

void ransacFitPlanesSample(const std::string &test_file_path, float thr, int desired_num_planes, int max_iters,
                           float grid_size)
{
    std::cout << "=========== Fit Planes ===========\n";
    if (thr <= 0 || desired_num_planes <= 0 || max_iters <= 0)
    {
        return;
    }
    cv::Mat point_cloud;
    if (read_point_cloud_ply_to_mat(test_file_path, point_cloud, false))
    {
        std::cout << "Read point cloud data successfully, point cloud size: " << point_cloud.size << std::endl;

        std::cout << "Start to fit the planes, point  cloud size  " << point_cloud.size;
        std::cout << ", desired_num_planes=" << desired_num_planes << ", thr=" << thr << ", max_iters=" << max_iters << ", grid size=" << grid_size << std::endl;
        std::vector<cv::Vec4f> planes_coeffs;
        cv::Mat labels;

        clock_t start = clock();
        cv::_3d::ransacFitPlanes(point_cloud, thr, max_iters, planes_coeffs, labels, desired_num_planes, grid_size);
        clock_t end = clock();

        std::cout << "Fit planes OK, the number of plane: " << planes_coeffs.size() << std::endl;
        std::cout << "Plane equations coefficients: ";
        for (const auto &item : planes_coeffs)
        {
            std::cout << item << " ";
        }
        std::cout << std::endl;
        std::cout << "Fit planes time cost: " << (1000.0 * (end - start)) / CLOCKS_PER_SEC << " ms" << std::endl;

        if (save_points_label(test_file_path + "-labels.txt", labels, false))
        {
            std::cout << "Save label successful, location: " << test_file_path + "-labels.txt\n";
        }
    }
    std::cout << "-----------------------------------\n\n";
}
} // _3d
} // cv::