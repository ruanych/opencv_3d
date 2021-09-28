#include "test_precomp.hpp"

static cv::Mat rac_point_cloud;
static std::vector<cv::Vec4f> models;
static const int multi_plane_desire_num = 2;

class RAC_FIT_PLANE : public ::testing::Test {
public:
    static void SetUpTestCase() {
        models.emplace_back(0, 1, 1, 0);
        models.emplace_back(1, 0, 1, 0);
        models.emplace_back(1, 2, 3, 4);
        point_cloud_generator(1000, 10000, 100, models, rac_point_cloud, 1);
    }

    std::vector<cv::Vec4f> coeffs;
    cv::Mat labels;
};

TEST_F(RAC_FIT_PLANE, SinglePlaneDetectionAccuray) {
    cv::_3d::ransacFitPlanes(rac_point_cloud, 0.1, 300, coeffs, labels, 1, 0);
    ASSERT_TRUE(compareCoeffs(coeffs[0], models[0]))
                                << "The plane coefficient is: " << coeffs[0] << std::endl;
}

TEST_F(RAC_FIT_PLANE, SinglePlaneDetectionWithNormAccuray) {
    cv::Vec3f norm = cv::Vec3f(models[1][0], models[1][1], models[1][2]);
    cv::_3d::ransacFitPlanes(rac_point_cloud, 0.1, 300, coeffs, labels, 1, 0, &norm);
    ASSERT_TRUE(compareCoeffs(coeffs[0], models[1]))
                                << "The plane coefficient is: " << coeffs[0] << std::endl;
}

TEST_F(RAC_FIT_PLANE, MultiPlaneDetectionAccuray) {
    cv::_3d::ransacFitPlanes(rac_point_cloud, 0.1, 300 * multi_plane_desire_num, coeffs, labels, multi_plane_desire_num,
                             0);
    for (int i = 0; i < multi_plane_desire_num; i++) {
        if (i >= coeffs.size())
            break;
        ASSERT_TRUE(compareCoeffs(coeffs[i], models[i]))
                                    << "The plane [" << i << "] coefficient is: " << coeffs[i] << std::endl;
    }
}

TEST_F(RAC_FIT_PLANE, MultiPlaneDetectionWithNormAccuray) {
    cv::Vec3f norm[multi_plane_desire_num];
    for (int i = multi_plane_desire_num - 1, j = 0; i >= 0; i--, j++) {
        norm[j] = cv::Vec3f(models[i][0], models[i][1], models[i][2]);
    }
    cv::_3d::ransacFitPlanes(rac_point_cloud, 0.1, 300 * multi_plane_desire_num, coeffs, labels, multi_plane_desire_num,
                             0,
                             norm);
    for (int i = multi_plane_desire_num - 1, j = 0; i >= 0; i--, j++) {
        if (j >= coeffs.size())
            break;
        ASSERT_TRUE(compareCoeffs(coeffs[j], models[i]))
                                    << "The plane [" << j << "]'s coefficient is: " << coeffs[j] << std::endl;
    }
}