#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include <sensors_calib/sensors_calib.hpp>

namespace
{
using PointCloudType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
}  // namespace
std::ofstream result_file;

void removeExtraLines(const std::string& file1, const std::string& file2) {
    std::ifstream fileStream1(file1);
    std::ifstream fileStream2(file2);

    if (!fileStream1 || !fileStream2) {
        std::cerr << "Error: Unable to open files." << std::endl;
        return;
    }

    std::vector<std::string> lines1;
    std::vector<std::string> lines2;
    std::string line;

    // Read lines from file1
    while (std::getline(fileStream1, line)) {
        lines1.push_back(line);
    }

    // Read lines from file2
    while (std::getline(fileStream2, line)) {
        lines2.push_back(line);
    }

    fileStream1.close();
    fileStream2.close();

    // Compare number of lines
    if (lines1.size() > lines2.size()) {
        std::ofstream outFile(file1);
        for (int i = 0; i < lines2.size(); i++) {
            outFile << lines1[i] << std::endl;
        }
        outFile.close();
        std::cout << "File 1 had more lines. Removed extra lines from file 1." << std::endl;
    } else if (lines1.size() < lines2.size()) {
        std::ofstream outFile(file2);
        for (int i = 0; i < lines1.size(); i++) {
            outFile << lines2[i] << std::endl;
        }
        outFile.close();
        std::cout << "File 2 had more lines. Removed extra lines from file 2." << std::endl;
    } else {
        std::cout << "Both files had the same number of lines. No action taken." << std::endl;
    }
}


int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/calibration/handler/data]" << std::endl;
        return EXIT_FAILURE;
    }

    std::string file1 = "/home/nuc/automatic_lidar_camera_calibration-master/data/samples/images.txt";
    std::string file2 = "/home/nuc/automatic_lidar_camera_calibration-master/data/samples/point_clouds.txt";
    removeExtraLines(file1, file2);

    const std::string PARAM_PATH = argv[1];

    perception::CalibrationHandlerParam param = perception::getCalibrationHandlerParam(PARAM_PATH);
    perception::CalibrationHandler<PointCloudType>::Ptr calibrationHandler(
        new perception::CalibrationHandler<PointCloudType>(param));

    auto transform = calibrationHandler->optimize();
    const auto visualizedImgs = calibrationHandler->drawPointCloudOnImagePlane(transform);
    const auto projectedClouds = calibrationHandler->projectOnPointCloud(transform);

    printf("x: %f[m], y: %f[m], z: %f[m], r: %f[deg], p: %f[deg], y_deg: %f[deg]\n", transform(0), transform(1),
           transform(2), transform(3) * boost::math::double_constants::radian,
           transform(4) * boost::math::double_constants::radian, transform(5) * boost::math::double_constants::radian);

    for (std::size_t i = 0; i < visualizedImgs.size(); ++i) {
        const auto& curImg = visualizedImgs[i];
        cv::imwrite("/home/nuc/automatic_lidar_camera_calibration-master/data/result/img" + std::to_string(i) + ".png", curImg);
    }

    for (std::size_t i = 0; i < projectedClouds.size(); ++i) {
        const auto& curCloud = projectedClouds[i];
        pcl::io::savePCDFileASCII("/home/nuc/automatic_lidar_camera_calibration-master/data/result/cloud" + std::to_string(i) + ".pcd", *curCloud);
    }
    Eigen::Affine3d affine = perception::toAffine(transform);
    result_file.open("/home/nuc/automatic_lidar_camera_calibration-master/data/result/transform.txt");
    if (result_file.is_open()) {
        result_file << "x, y, z, roll, pitch, yaw is: " << "\n"
                    << transform(0) << "\n"
                    << transform(1) << "\n"
                    << transform(2) << "\n"
                    << transform(3) * boost::math::double_constants::radian << "\n"
                    << transform(4) * boost::math::double_constants::radian << "\n"
                    << transform(5) * boost::math::double_constants::radian << "\n"
                    <<"Transform Matrix is: " << "\n"
                    << affine.matrix();          
    } else {
        std::cout << "Unable to open result file" << std::endl;
    }
    result_file.close();
    return EXIT_SUCCESS;
}
