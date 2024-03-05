#include "test_utils.hpp"

bool vecStd(cv::Vec4f &v) {
    float denominator = cv::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    if (denominator == 0)
        return false;
    v /= denominator;
    return true;
}

bool compareCoeffs(cv::Vec4f m, cv::Vec4f n) {
    if (!vecStd(m) | !vecStd(n)) return m == n;
    cv::Vec4f tmp = m - n;
    float var = tmp.dot(n);
    return var <= 1e-3;
}

bool comparePoints(const cv::Mat &m, int rm, const cv::Mat &n, int rn) {
    cv::Mat diff = m.row(rm) != n.row(rn);
    return cv::countNonZero(diff) == 0;
}

void
point_cloud_generator(float size, int point_num, int noise_num, std::vector<cv::Vec4f> models, cv::Mat &point_cloud,
                      int seed) {
    point_cloud = cv::Mat(point_num + noise_num, 3, CV_32F);
    cv::RNG rng(seed);

    int res_num = point_num;
    int models_size = models.size();
    int per_num, ran_num;
    int idx = 0;
    cv::Vec4f model;
    float x, y, z;

    for (int i = 0; i < models_size; i++) {
        per_num = (int) (res_num / (models_size - i));
        ran_num = (int) (per_num * 1.5);
        if (i == models_size - 1)
            ran_num = res_num;
        res_num -= ran_num;
        model = models.at(i);
        for (int j = 0; j < ran_num; j++) {
            x = rng.uniform(-size, size);
            y = rng.uniform(-size, size);
            z = -(model[0] * x + model[1] * y + model[3]) / model[2];
            if (z > size || z < -size) {
                j--;
                continue;
            }
            point_cloud.at<float>(idx, 0) = x;
            point_cloud.at<float>(idx, 1) = y;
            point_cloud.at<float>(idx, 2) = z;
            idx++;
        }
    }

    for (int i = 0; i < noise_num; i++) {
        point_cloud.at<float>(idx, 0) = rng.uniform(-size, size);
        point_cloud.at<float>(idx, 1) = rng.uniform(-size, size);
        point_cloud.at<float>(idx, 2) = rng.uniform(-size, size);
        idx++;
    }
}