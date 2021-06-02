#include "opencv2/opencv_3d/sampling.hpp"
#include <unordered_map>
#include <cmath>

namespace cv {
    namespace _3d {

void voxelGrid(cv::InputArray &input_pts, const float length, const float width,
               const float height, cv::OutputArray &sampled_pts)
{
    CV_CheckGT(length, 0.0f, "Invalid length of grid");
    CV_CheckGT(width, 0.0f, "Invalid width of grid");
    CV_CheckGT(height, 0.0f, "Invalid height of grid");

    // Get input point cloud data
    cv::Mat ori_pts;
    if (input_pts.isMat())
    {
        ori_pts = input_pts.getMat();
        if (ori_pts.channels() != 1)
            ori_pts = ori_pts.reshape(1, static_cast<int>(ori_pts.total())); // Convert to single channel
        if (ori_pts.cols != 3 && ori_pts.rows == 3)
            cv::transpose(ori_pts, ori_pts);

        CV_CheckEQ(ori_pts.cols, 3, "Invalid dimension of point cloud");
        if (ori_pts.type() != CV_32F)
            ori_pts.convertTo(ori_pts, CV_32F);// Use float to store data
    }
    else
    {
        ori_pts = cv::Mat(static_cast<int>(ori_pts.total()), 3, CV_32F, ori_pts.data);
    }

    const int ori_pts_size = ori_pts.rows;

    float *const ori_pts_ptr = (float *) ori_pts.data;

    // Compute the minimum and maximum bounding box values
    float x_min, x_max, y_min, y_max, z_min, z_max;
    x_max = x_min = ori_pts_ptr[0];
    y_max = y_min = ori_pts_ptr[1];
    z_max = z_min = ori_pts_ptr[2];

    for (int i = 1; i < ori_pts_size; ++i)
    {
        float *const ptr_base = ori_pts_ptr + 3 * i;
        float x = *(ptr_base), y = *(ptr_base + 1), z = *(ptr_base + 2);

        if (x_min > x) x_min = x;
        if (x_max < x) x_max = x;

        if (y_min > y) y_min = y;
        if (y_max < y) y_max = y;

        if (z_min > z) z_min = z;
        if (z_max < z) z_max = z;
    }

    typedef long long keyType;
    std::unordered_map<keyType, std::vector<int>> grids;

//            int init_size = ori_pts_size * 0.02;
//            grids.reserve(init_size);

    // Divide points into different grids

    long offset_y = static_cast<long>(cvCeil((x_max - x_min) / length));
    long offset_z = offset_y * static_cast<long>(cvCeil((y_max - y_min) / width));

    for (int i = 0; i < ori_pts_size; ++i)
    {
        int ii = 3 * i;
        long hx = static_cast<long>((ori_pts_ptr[ii] - x_min) / length);
        long hy = static_cast<long>((ori_pts_ptr[ii + 1] - y_min) / width);
        long hz = static_cast<long>((ori_pts_ptr[ii + 2] - z_min) / height);
        // Convert three-dimensional coordinates to one-dimensional coordinates key
        // Place the stacked three-dimensional grids(boxes) into one dimension (placed in a straight line)
        keyType key = hx + hy * offset_y + hz * offset_z;

        std::unordered_map<keyType, std::vector<int>>::iterator iter = grids.find(key);
        if (iter == grids.end())
            grids[key] = {i};
        else
            iter->second.push_back(i);
    }

    const int pts_new_size = static_cast<int>(grids.size());
    sampled_pts.create(pts_new_size, 3, CV_32F);

    float *const sampling_pts_ptr = (float *) sampled_pts.getMat().data;

    // Take out the points in the grid and calculate the point closest to the centroid
    std::unordered_map<keyType, std::vector<int>>::iterator grid_iter = grids.begin();

    for (int label_id = 0; label_id < pts_new_size; ++label_id, ++grid_iter)
    {
        // Calculate the centroid position
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (const int &item : grid_iter->second)
        {
            float *const ptr_base = ori_pts_ptr + 3 * item;
            sum_x += *(ptr_base);
            sum_y += *(ptr_base + 1);
            sum_z += *(ptr_base + 2);
        }
        int grid_pts_cnt = static_cast<int>(grid_iter->second.size());
        float centroid_x = sum_x / grid_pts_cnt, centroid_y = sum_y / grid_pts_cnt, centroid_z =
                sum_z / grid_pts_cnt;

        // Find the point closest to the centroid
        float *sampled_ptr_base;
        float min_dist_square = FLT_MAX;
        for (const int &item : grid_iter->second)
        {
            float *const ptr_base = ori_pts_ptr + item * 3;
            float x = *(ptr_base), y = *(ptr_base + 1), z = *(ptr_base + 2);

            float dist_square = (x - centroid_x) * (x - centroid_x) +
                                (y - centroid_y) * (y - centroid_y) +
                                (z - centroid_z) * (z - centroid_z);
            if (dist_square < min_dist_square)
            {
                min_dist_square = dist_square;
                sampled_ptr_base = ptr_base;
            }
        }

        float *const new_pts_ptr_base = sampling_pts_ptr + 3 * label_id;
        *(new_pts_ptr_base) = *sampled_ptr_base;
        *(new_pts_ptr_base + 1) = *(sampled_ptr_base + 1);
        *(new_pts_ptr_base + 2) = *(sampled_ptr_base + 2);
    }

} // voxelGrid()

    } // _3d::
} // cv::