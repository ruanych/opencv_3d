
#ifndef OPENCV_OPENCV_3D_UTILS_H
#define OPENCV_OPENCV_3D_UTILS_H

#include <fstream>
#include <opencv2/opencv.hpp>

namespace cv {
    namespace _3d {

/**
 * Save point cloud label
 *
 * @param file_path  File save path
 * @param labels  Mat for storing label
 * @param sync_io  IO synchronization
 * @return  true if saved successfully, otherwise false
 */
bool savePointsLabel(const std::string &file_path, cv::InputArray &labels, bool sync_io = false);

/**
 * Save point cloud to .ply file
 *
 * @param file_path  ply file output path
 * @param input_pts   Point cloud in mat
 * @param sync_io  IO synchronization
 * @return  true if saved successfully, otherwise false
 */
bool savePointCloudPly(const std::string &file_path, cv::InputArray &input_pts, bool sync_io = false);


/**
 *  Read point cloud data from ply file
 *
 * @param file_path  Point cloud data file path
 * @param output_pts  Point cloud (output)
 * @param sync_io  IO synchronization
 * @return  true if read data successfully, otherwise false
 */
bool readPointCloudPlyToMat(const std::string &file_path, cv::OutputArray &output_pts, bool sync_io = false);

    } // _3d::
} // cv::

#endif //OPENCV_OPENCV_3D_UTILS_H
