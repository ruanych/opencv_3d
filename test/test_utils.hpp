#ifndef OPENCV_3D_TEST_UTILS_HPP
#define OPENCV_3D_TEST_UTILS_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

bool compareCoeffs(cv::Vec4f, cv::Vec4f);

bool comparePoints(const cv::Mat&, int, const cv::Mat&, int);

void point_cloud_generator(float size, int point_num, int noise_num, std::vector<cv::Vec4f> models, cv::Mat &point_cloud, int seed = 0);

#endif //OPENCV_3D_TEST_UTILS_HPP
