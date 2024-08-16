#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <sensors_calib/sensors_calib.hpp>
//#include <msg_all/msg/image_frame.hpp>

namespace
{
using PointCloudType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
}  // namespace

std::ofstream images_file;
std::ofstream point_clouds_file;

std::string pkg_loc;

static cv::Mat D (5,1,cv::DataType<double>::type); 
static cv::Mat K (3,3,cv::DataType<double>::type); 

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

void getCameraInfo(void) {
    rapidjson::Document jsonDoc = perception::readFromJsonFile(pkg_loc + "/data/samples/camera_info.json");
    const rapidjson::Value& intri = jsonDoc["K"];
    const rapidjson::Value& distort_coeffs = jsonDoc["distort_coeffs"];
    if (!intri.IsArray()) {
        throw std::runtime_error("failed to get K");
    }

    if (static_cast<int>(intri.Size()) != 9) {
        throw std::runtime_error("invalid K");
    }

    if (!distort_coeffs.IsArray()) {
        throw std::runtime_error("failed to get distort_coeffs");
    }

    if (static_cast<int>(distort_coeffs.Size()) != 5) {
        throw std::runtime_error("invalid distort_coeffs");
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K.at<double>(i, j) = intri[static_cast<rapidjson::SizeType>(i * 3 + j)].GetDouble();
        }
    }

    for (int i = 0; i < 5; i++) {
        D.at<double>(i,0) = distort_coeffs[static_cast<rapidjson::SizeType>(i)].GetDouble();
    }
}

/* void saveImage(const msg_all::msg::ImageFrame::SharedPtr msg, int count) {
    cv::Mat image; 
    cv::Mat image_undistort;
    
    int height = msg->height;
    int width = msg->width;
    cv::Mat cv_yuv(height, width, CV_8UC2, msg->data.data());
    cv::cvtColor(cv_yuv, image, cv::COLOR_YUV2BGR_YUY2);

    cv::undistort(image, image_undistort, K, D);
    std::string file_path = pkg_loc + "/data/images/" + std::to_string(count) + ".jpg";
    cv::imwrite(file_path, image_undistort);
    std::cout << "capture " << count+1 << " image data" << std::endl;

    if (images_file.is_open()) {
        images_file << file_path << std::endl;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file");
    }
} */

void saveImage(const sensor_msgs::msg::Image::SharedPtr msg, int count) {
    cv::Mat image; 
    cv::Mat image_undistort;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }
    image  = cv_ptr->image;
    cv::undistort(image, image_undistort, K, D);
    std::string file_path = pkg_loc + "/data/images/" + std::to_string(count) + ".jpg";
    cv::imwrite(file_path, image_undistort);
    std::cout << "capture " << count+1 << " image data" << std::endl;

    if (images_file.is_open()) {
        images_file << file_path << std::endl;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file");
    }
}

void savePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int count) {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    PointCloud cloud;
    pcl::fromPCLPointCloud2(pcl_pc, cloud);

    std::string file_path = pkg_loc + "/data/point_clouds/" + std::to_string(count) + ".pcd";
    pcl::io::savePCDFileASCII(file_path, cloud);
    std::cout << "capture " << count+1 << " point_coud data" << std::endl;

    if (point_clouds_file.is_open()) {
        point_clouds_file << file_path << std::endl;

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open file");
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/calibration/handler/data]" << std::endl;
        return EXIT_FAILURE;
    }
    const std::string PARAM_PATH = argv[1];
    // 查找 "data" 子字符串的位置
    size_t pos = PARAM_PATH.find("/data/");
    pkg_loc = PARAM_PATH.substr(0, pos);

    getCameraInfo();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensors_calib_ros2_node");
    int capture_count = 0;
/*     auto image_sub = node->create_subscription<msg_all::msg::ImageFrame>(
        "image_frame_topic", 10, [&](const msg_all::msg::ImageFrame::SharedPtr msg) {
            // 保存图像
            static int count = 0;
            saveImage(msg, count);
            capture_count = count;
            count++;
        }); */

    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "kitti/image/color/left", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
            // Save image when 'p' is pressed
            static int count = 0;
            saveImage(msg, count);
            capture_count = count;
            count++;
        });

    auto point_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "kitti/point_cloud", 10, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // Save point cloud when 'p' is pressed
            static int count = 0;
            savePointCloud(msg, count);
            capture_count = count;
            count++;
        });
    std::string file1 = pkg_loc + "/data/samples/images.txt";
    std::string file2 = pkg_loc + "/data/samples/point_clouds.txt";    
    images_file.open(file1);
    point_clouds_file.open(file2);

    char key_press;
    while (rclcpp::ok()) {
        std::cout << "press 'p' to capture the data or press 'e' to exit" << std::endl;
        std::cin >> key_press;
        if (key_press == 'p') {
            rclcpp::spin_some(node);
        } else if (key_press == 'e') {
            break;
        }
    } 
/*     // Create a timer to trigger the callback function every 1 second
    while (rclcpp::ok() && capture_count < 5) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::spin_some(node);
    } */

    images_file.close();
    point_clouds_file.close();
    removeExtraLines(file1, file2);

    std::cout<<"continue to calib"<<std::endl;

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
        cv::imwrite(pkg_loc + "/data/result/img" + std::to_string(i) + ".png", curImg);
    }

    for (std::size_t i = 0; i < projectedClouds.size(); ++i) {
        const auto& curCloud = projectedClouds[i];
        pcl::io::savePCDFileASCII(pkg_loc + "/data/result/cloud" + std::to_string(i) + ".pcd", *curCloud);
    }
    Eigen::Affine3d affine = perception::toAffine(transform);
    std::ofstream result_file;
    result_file.open(pkg_loc + "/data/result/transform.txt");
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
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

