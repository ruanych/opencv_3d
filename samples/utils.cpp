#include "utils.hpp"

namespace cv {
    namespace _3d {

bool save_points_label(const std::string &file_path, cv::InputArray &labels, bool sync_io)
{
    cv::Mat labels_m = labels.getMat();

    int size = labels_m.rows;
    if (size == 0) return false;

    std::ios::sync_with_stdio(sync_io);
    std::ofstream ofs(file_path);

    if (!ofs.is_open())
    {
        std::cerr << "ofstream open file error!\n";
        return false;
    }

    ofs.clear();

    int *const pts_ptr = (int *) labels_m.data;

    for (int i = 0; i < size; ++i) ofs << pts_ptr[i] << "\n";

    ofs.flush();
    ofs.close();
    std::ios::sync_with_stdio(true);
    return true;
} // save_points_label()


bool save_point_cloud_ply(const std::string &file_path, cv::InputArray &input_pts, bool sync_io)
{
    cv::Mat pts_m = input_pts.getMat();
    int size = pts_m.rows;
    if (size == 0)  return false;

    std::ios::sync_with_stdio(sync_io);
    std::ofstream ofs(file_path);

    if (!ofs.is_open())
    {
        std::cerr << "ofstream open file error!\n";
        return false;
    }
    ofs.clear();

    std::string head = "ply\n"
                       "format ascii 1.0\n"
                       "comment Created by OPENCV_3D\n"
                       "element vertex " + std::to_string(size) + "\n"
                                                                  "property float x\n"
                                                                  "property float y\n"
                                                                  "property float z\n"
                                                                  "end_header\n";
    ofs << head;

    float *const pts_ptr = (float *) pts_m.data;

//    ofs << std::fixed << std::setprecision(2);

    for (int i = 0; i < size; ++i)
    {
        float *ptr_base = pts_ptr + 3 * i;
        ofs << *(ptr_base) << " " << *(ptr_base + 1) << " " << *(ptr_base + 2) << "\n";
    }

    ofs.flush();
    ofs.close();
    std::ios::sync_with_stdio(true);
    return true;
} // save_point_cloud_ply()

bool
read_point_cloud_ply_to_mat(const std::string &file_path, cv::OutputArray &output_pts, bool sync_io)
{
    std::ios::sync_with_stdio(sync_io);

    std::ifstream ifs(file_path);
    if (!ifs.is_open())
    {
        std::cerr << "ifstream open point cloud data file error!\n";
        return false;
    }

    std::string head;
    int size = 0;
    while (head != "end_header")
    {
        if (head == "vertex") ifs >> size;
        ifs >> head;
    }

    if (size < 0)
    {
        std::cerr << "File read exception\n";
        return false;
    }

    output_pts.create(size, 3, CV_32F);

    float *pts_ptr = (float *) output_pts.getMat().data;
    size *= 3;
    for (int i = 0; i < size; ++i)  ifs >> pts_ptr[i];

    std::ios::sync_with_stdio(true);
    return true;
} // read_point_cloud_ply_to_mat()

    } // _3d::
} // cv::