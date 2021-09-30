#include "test_precomp.hpp"

static float sampling_point_cloud_info[24] = {
        1, 1, 1, 0, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 0, 0, 1, 0, 1,
        1, 1, 0, 0, 0, 0
};

static cv::Mat sampling_point_cloud(8, 3, CV_32F, sampling_point_cloud_info);

class SAMPLING : public ::testing::Test {
public:
    cv::Mat sampled_pts;
};


TEST_F(SAMPLING, VoxelGridFilterSampling) {
    cv::_3d::voxelGrid(sampling_point_cloud, 1.1, 1.1, 1.1, sampled_pts);
    ASSERT_TRUE(sampled_pts.rows == 1) << "Sampling should only select one point, but " << sampled_pts.rows
                                       << " points are selected." << std::endl;
    bool tmp = false;
    for (int i = 0; i < sampling_point_cloud.rows; i++) {
        if (comparePoints(sampling_point_cloud, i, sampled_pts, 0)) {
            tmp = true;
            break;
        }
    }
    ASSERT_TRUE(tmp) << "The points created by sampling are not in the original point cloud: " << sampled_pts
                     << std::endl;
}

TEST_F(SAMPLING, RandomSampling) {
    cv::_3d::randomSampling(sampling_point_cloud, 1, sampled_pts);
    ASSERT_TRUE(sampled_pts.rows == 1) << "Sampling should only select one point, but " << sampled_pts.rows
                                       << " points are selected." << std::endl;
    bool tmp = false;
    for (int i = 0; i < sampling_point_cloud.rows; i++) {
        if (comparePoints(sampling_point_cloud, i, sampled_pts, 0)) {
            tmp = true;
            break;
        }
    }
    ASSERT_TRUE(tmp) << "The points created by sampling are not in the original point cloud: " << sampled_pts
                     << std::endl;
}

TEST_F(SAMPLING, FarthestPointSampling) {
    cv::_3d::farthestPointSampling(sampling_point_cloud, 2, sampled_pts);
    cv::Mat check = sampled_pts.row(0) + sampled_pts.row(1);
    bool tmp = check.at<float>(0, 0) == check.at<float>(0, 1) == check.at<float>(0, 2) == 1;
    ASSERT_TRUE(tmp) << "FPS did not get the farthest point in the point cloud.";
}

