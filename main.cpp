#include <iostream>
#include "samples/samples.hpp"

int main(int argc, char *argv[])
{

    if (argc > 1){
        std::string test_file_path = argv[1];
        cv::_3d::sampling_sample(test_file_path);
    }

    cv::_3d::total_least_squares_sample();

    return 0;
}
