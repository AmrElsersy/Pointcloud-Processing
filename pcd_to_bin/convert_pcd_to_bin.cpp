#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <iostream>
#include <pcl/io/auto_io.h>


namespace fs = std::filesystem;

void savePointCloudBinary(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, std::string savePath)
{
    std::cout << pointcloud->size() << " - " <<  pointcloud->points.size() << " @ " << savePath << std::endl ;
    auto file = std::fstream(savePath, std::ios::out | std::ios::binary);

    for (auto point : pointcloud->points)
    {
        file.write((char*)&point.x, sizeof(point.x));
        file.write((char*)&point.y, sizeof(point.y));
        file.write((char*)&point.z, sizeof(point.z));
        file.write((char*)&point.intensity, sizeof(point.intensity));
    }
    file.close();
}


int main(int argc, char** argv)
{
    // std::string root = "../../../training/pointclouds/";
    std::string root = "../../../testing111/pointclouds";
    std::string saveRoot;

    if (argc == 2)
        root = argv[1];

    if (root[root.size()-1] == '/')
        saveRoot = root.substr(0, root.size()-1) + "_bin/";
    else
        saveRoot = root + "_bin/";

    if (!std::filesystem::is_directory(saveRoot))
        std::filesystem::create_directory(saveRoot);

    for (const auto & entry : fs::directory_iterator(root))
    {
        // extract the name of the pcd file
        std::string path = entry.path();
        int pos = path.find_last_of('.');
        int slashPos = path.find_last_of('/');
        std::string savePath = path.substr(slashPos+1, pos-slashPos-1) + ".bin";

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(path, *pointcloud);
        savePointCloudBinary(pointcloud, saveRoot + savePath);
    }
    return 0;
}
