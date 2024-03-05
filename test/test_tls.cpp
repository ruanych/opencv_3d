#include "test_precomp.hpp"

static float tls_point_cloud_info[45] = {
        1, 2, 1, 2, 4, 2, 3, 6, 3,
        1, 1, 0, 0, 1, 1, 1, 0, 1,
        2, 2, 0, 0, 2, 2, 2, 0, 2,
        3, 3, 0, 0, 3, 3, 3, 0, 3,
        1, 2, 0, 0, 1, 2, 1, 0, 2
};
static cv::Mat tls_point_cloud(15, 3, CV_32F, tls_point_cloud_info);

class TLS : public ::testing::Test {
public:
    static void SetUpTestCase() {}
};


TEST_F(TLS, ThreePointsPlane) {
    cv::Vec4f coeff;
    std::vector<int> idx = {3, 4, 5};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(1, 1, 1, -2))) << "Coefficient is: " << coeff << std::endl;
}

TEST_F(TLS, ThreePointsLine) {
    cv::Vec4f coeff;
    std::vector<int> idx = {0, 1, 2};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 0, 0, 0))) << "Coefficient is: " << coeff << std::endl;
}

TEST_F(TLS, LinePerpendiularAxis) {
    cv::Vec4f coeff;
    std::vector<int> idx = {3, 6, 9};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 0, 0, 0))) << "Coefficient is: " << coeff << std::endl;
    idx = {4, 7, 10};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 0, 0, 0))) << "Coefficient is: " << coeff << std::endl;
    idx = {5, 8, 11};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 0, 0, 0))) << "Coefficient is: " << coeff << std::endl;
}

TEST_F(TLS, PlaneInAxis) {
    cv::Vec4f coeff;
    std::vector<int> idx = {3, 6, 12};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 0, 1, 0))) << "Coefficient is: " << coeff << std::endl;
    idx = {4, 7, 13};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(1, 0, 0, 0))) << "Coefficient is: " << coeff << std::endl;
    idx = {5, 8, 14};
    cv::_3d::totalLeastSquaresPlaneEstimate(tls_point_cloud, idx, coeff);
    ASSERT_TRUE(compareCoeffs(coeff, cv::Vec4f(0, 1, 0, 0))) << "Coefficient is: " << coeff << std::endl;
}
