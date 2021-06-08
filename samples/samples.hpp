
#ifndef OPENCV_OPENCV_3D_SAMPLES_HPP
#define OPENCV_OPENCV_3D_SAMPLES_HPP
namespace cv {
namespace _3d {

void voxelGridSamplingSample(const std::string &test_file_path, float grid_size);

void randomSamplingSample(const std::string &test_file_path, float sampled_scale);

void farthestPointSamplingSample(const std::string &test_file_path, float sampled_scale);

void totalLeastSquaresSample();

void ransacFitPlanesSample(const std::string &test_file_path, float thr, int desired_num_planes, int max_iters, float grid_size);

} // _3d::
} // cv::


#endif //OPENCV_OPENCV_3D_SAMPLES_HPP
