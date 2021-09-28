#include <iostream>
#include "samples/samples.hpp"

int main(int argc, char *argv[]) {

    using namespace std;
    while (true) {
        cout << "*****************************************\n";
        cout << "0. Exit this program\n";
        cout << "1. RANSAC fit planes\n";
        cout << "2. Total least squares\n";
        cout << "3. Voxel grid filter sampling\n";
        cout << "4. Random sampling\n";
        cout << "5. Farthest point sampling\n";
        cout << "Please select the sample to run:(1~5): ";

        int num, max_iters, desired_num_planes;
        float grid_size, thr, sampled_scale;
        cin >> num;

        string test_file_path;
        if (argc > 1) {
            test_file_path = argv[1];
        } else if (num != 2 && num != 0) {
            cout << "Please enter the path of the point cloud file (.ply): ";
            cin >> test_file_path;
        }


        switch (num) {
            case 0:
                cout << "Exit now";
                return 0;
            case 1:
                cout << "Please enter inlier point threshold(e.g. 0.5): ";
                cin >> thr;
                cout << "Please enter desired number of planes(e.g. 2): ";
                cin >> desired_num_planes;
                cout << "Please enter RANSAC max iterations(e.g. 500): ";
                cin >> max_iters;
                cout
                        << "Please enter the grid size length(e.g. 0.2), Less than or equal to 0 means no down-sampling is used: ";
                cin >> grid_size;
                cv::_3d::ransacFitPlanesSample(test_file_path, thr, desired_num_planes, max_iters, grid_size);
                break;
            case 2:
                cv::_3d::totalLeastSquaresSample();
                break;
            case 3:
                cout << "Please enter the grid size length(e.g. 0.2): ";
                cin >> grid_size;
                cv::_3d::voxelGridSamplingSample(test_file_path, grid_size);
                break;
            case 4:
                cout << "Please enter the Please input sampling scale(e.g. 0.2): ";
                cin >> sampled_scale;
                cv::_3d::randomSamplingSample(test_file_path, sampled_scale);
                break;
            case 5:
                cout << "Please enter the Please input sampling scale(e.g. 0.2): ";
                cin >> sampled_scale;
                cv::_3d::farthestPointSamplingSample(test_file_path, sampled_scale);
                break;
            default:
                cout << "This sample does not exist.\n";
        }

    }
}
