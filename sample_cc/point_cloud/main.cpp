#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = boost::filesystem;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCSVFiles(const std::string& directory) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".csv") {
            std::ifstream file(entry.path().string());
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << entry.path() << std::endl;
                continue;
            }
            std::string line;
            if (std::getline(file, line)) {
                std::cout << "Skipping header in " << entry.path().filename().string() << ": " << line << std::endl;
            }
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string x_str, y_str, z_str;
                if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',') || !std::getline(ss, z_str, ',')) {
                    continue;
                }
                pcl::PointXYZ point;
                try {
                    point.x = std::stof(x_str);
                    point.y = std::stof(y_str);
                    point.z = std::stof(z_str);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid data in " << entry.path().filename().string() << ": " << line << std::endl;
                    continue;
                }
                cloud->points.push_back(point);
            }
            file.close();
            std::cout << "Loaded " << entry.path().filename().string() << " with " << cloud->points.size() << " points." << std::endl;
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}
Eigen::Affine3f loadCameraPosition() {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    std::ifstream file("camera_position.txt");
    if (!file.is_open()) {
        std::cerr << "Failed to open camera position file." << std::endl;
        return Eigen::Affine3f::Identity();
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            file >> pose(i, j);
        }
    }
    file.close();
    std::cout << "Camera position loaded from camera_position.txt" << std::endl;
    return Eigen::Affine3f(pose); // Convert Matrix4f to Affine3f
}


int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <directory>" << std::endl;
        return -1;
    }

    std::string directory = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadCSVFiles(directory);
    if (cloud->empty()) {
        std::cerr << "No points loaded. Please check the directory and CSV files." << std::endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->setCameraPosition(
        loadCameraPosition().matrix()(0, 3), loadCameraPosition().matrix()(1, 3), loadCameraPosition().matrix()(2, 3), // カメラの位置
        0, 0, 0, // 注視点
        0, 0, 1); // カメラの上向きベクトル

    viewer->saveScreenshot("output.png");
    std::cout << "Screenshot saved as output.png" << std::endl;

    return 0;
}

