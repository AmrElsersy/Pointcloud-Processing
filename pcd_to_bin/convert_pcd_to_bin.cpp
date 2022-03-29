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


int main()
{
    std::string root = "/home/amrelsersy/oustar/oustar/pointclouds";
    for (const auto & entry : fs::directory_iterator(root))
    {
        // extract the name of the pcd file
        std::string path = entry.path();
        int pos = path.find_last_of('.');
        std::string savePath = path.substr(0, pos) + ".bin";

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(path, *pointcloud);
        savePointCloudBinary(pointcloud, savePath);

        exit(0);
    }
    return 0;
}
