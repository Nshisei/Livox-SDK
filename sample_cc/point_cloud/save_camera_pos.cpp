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

void saveCameraPosition(pcl::visualization::PCLVisualizer::Ptr viewer) {
    std::ofstream file("camera_position.txt");
    if (!file.is_open()) {
        std::cerr << "Failed to open camera position file for writing." << std::endl;
        return;
    }

    Eigen::Affine3f pose = viewer->getViewerPose();

    // 位置、注視点、上向きベクトルを抽出して保存
    Eigen::Vector3f position = pose.translation();
    Eigen::Vector3f look_at = position + pose.linear().col(2); // Z軸方向が注視点
    Eigen::Vector3f up_vector = pose.linear().col(1); // Y軸方向が上向き

    file << position.x() << " " << position.y() << " " << position.z() << std::endl;
    file << look_at.x() << " " << look_at.y() << " " << look_at.z() << std::endl;
    file << up_vector.x() << " " << up_vector.y() << " " << up_vector.z() << std::endl;

    file.close();
    std::cout << "Camera position saved to camera_position.txt" << std::endl;
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

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);

        if (viewer->wasStopped()) {
            saveCameraPosition(viewer);
            break;
        }
    }

    return 0;
}

