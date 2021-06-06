#include <iostream>
#include "samples/samples.hpp"

int main(int argc, char *argv[])
{

    using namespace std;
    cout << "*****************************************\n";
    cout << "1. Voxel grid filter sampling\n";
    cout << "2. Total least squares\n";
    cout << "3. RANSAC fit planes\n";
    cout << "Please select the sample to run:(1~3):";

    int num, max_iters, desired_num_planes;
    float grid_size, thr;
    cin >> num;

    string test_file_path;
    if (argc > 1)
    {
        test_file_path = argv[1];
    }
    else if (num == 1 || num == 3)
    {
        cout << "Please enter the path of the point cloud file (.ply): ";
        cin >> test_file_path;
    }


    switch (num)
    {
        case 1:
            cout << "Please enter the grid size length(e.g. 0.2): ";

            cin >> grid_size;
            cv::_3d::sampling_sample(test_file_path, grid_size);
            break;
        case 2:
            cv::_3d::total_least_squares_sample();
            break;
        case 3:
            cout << "Please enter inlier point threshold(e.g. 0.5): ";
            cin >> thr;
            cout << "Please enter desired number of planes(e.g. 2): ";
            cin >> desired_num_planes;
            cout << "Please enter RANSAC max iterations(e.g. 500): ";
            cin >> max_iters;
            cout << "Please enter the grid size length(e.g. 0.2), Less than or equal to 0 means no down-sampling is used: ";
            cin >> grid_size;
            cv::_3d::ransacFitPlanesSample(test_file_path, thr, desired_num_planes, max_iters, grid_size);
            break;
        default:
            cout << "This sample does not exist.\n";
    }

    return 0;
}
