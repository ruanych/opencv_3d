
#ifndef OPENCV_OPENCV_3D_SAMPLES_HPP
#define OPENCV_OPENCV_3D_SAMPLES_HPP
namespace cv {
namespace _3d {

void sampling_sample(const std::string &test_file_path, float grid_size);

void total_least_squares_sample();

void ransacFitPlanesSample(const std::string &test_file_path, float thr, int desired_num_planes, int max_iters, float grid_size);

} // _3d::
} // cv::


#endif //OPENCV_OPENCV_3D_SAMPLES_HPP
