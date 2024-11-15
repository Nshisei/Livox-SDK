#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>  // Boost Filesystemをインクルード
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>
namespace fs = boost::filesystem;  // Boost Filesystemのエイリアス
// CSVファイルを読み込み、点群に変換する関数
pcl::PointCloud<pcl::PointXYZ>::Ptr loadCSVFiles(const std::string& directory) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ディレクトリ内のすべてのCSVファイルを読み込む
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".csv") {
            std::ifstream file(entry.path().string());  // Boostでパスの文字列取得
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << entry.path() << std::endl;
                continue;
            }

            std::string line;

            // 最初の行をスキップ（ヘッダー行）
            if (std::getline(file, line)) {
                std::cout << "Skipping header in " << entry.path().filename().string() << ": " << line << std::endl;
            }

            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string x_str, y_str, z_str;
                if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',') || !std::getline(ss, z_str, ',')) {
                    continue; // フォーマットに合わない行はスキップ
                }

                pcl::PointXYZ point;
                try {
                    point.x = std::stof(x_str);
                    point.y = std::stof(y_str);
                    point.z = std::stof(z_str);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid data in " << entry.path().filename().string() << ": " << line << std::endl;
                    continue; // 無効なデータがあればスキップ
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

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <directory>" << std::endl;
        return -1;
    }

    // 指定ディレクトリから点群データを読み込み
    std::string directory = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadCSVFiles(directory);

    if (cloud->empty()) {
        std::cerr << "No points loaded. Please check the directory and CSV files." << std::endl;
        return -1;
    }

    // PCLビジュアライザの設定
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();



    // ビジュアライザをループで表示
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}

