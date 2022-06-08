/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-07 14:38:02
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-06-07 16:15:36
 * @Description: Description
 */
#include <cmath>
#include <iostream>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <camera_models/CameraFactory.h>
#include <camera_models/PinholeCamera.h>

#include "DataLoader.hpp"
#include "pcl/visualization/point_cloud_color_handlers.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    std::string cfg_file;
    if (argv[1])
    {
        cfg_file = argv[1];
    }
    else
    {
        cfg_file = "/home/chrisliu/narwal_ws/src/CalculateImagesOverlapInHDMap/config/blender/blender.yaml";
    }

    Config config = loadConfigFromYamlFile(cfg_file);

    // TODO: pose跟image一一对应，遍历gt_data：取id读depth,去畸变，投成点云
    // 潜在问题：depth分辨率较高，是否导致点云文件过大？
    std::map<long long, Eigen::Isometry3d> gt_data = loadGTData(config.gt_file);

    // Generate random number in given range
    srand((unsigned int)time(nullptr)); //初始化种子为随机值

    // read ply file and visualize
    PointCloudT::Ptr cloud_map(new PointCloudT);

    bool skip = false;
    for (auto &it : gt_data)
    {
        if (skip)
        {
            skip = false;
            continue;
        }
        else
            skip = true;
        PointCloudT::Ptr current(new PointCloudT);
        std::string id_str = std::to_string(it.first);
        while (id_str.size() < 8)
            id_str = "0" + id_str;
        cout << "Read: " << config.depth_path + "/" + id_str + ".png" << endl;
        cv::Mat depth_img = cv::imread(config.depth_path + "/" + id_str + ".png", cv::IMREAD_UNCHANGED);
        for (int i = 0; i < depth_img.rows; i++)
        {
            for (int j = 0; j < depth_img.cols; j++)
            {
                if (depth_img.at<ushort>(i, j) == 0)
                {
                    continue;
                }

                // https://docs.blender.org/manual/zh-hant/dev/render/cycles/object_settings/cameras.html#fisheye
                // https://baike.baidu.com/item/%E7%90%83%E5%9D%90%E6%A0%87%E7%B3%BB/8315363?fr=aladdin
                double x = j - depth_img.cols / 2;
                double y = i - depth_img.rows / 2;
                double r = sqrt(x * x + y * y);
                double theta = (18.0 * r * 10.0 / depth_img.rows) / 180.0 * M_PI;
                double phi = acos(x / r);
                Eigen::Vector3d point_eigen;
                if (y > 0)
                {
                    point_eigen.x() = sin(theta) * cos(phi);
                    point_eigen.y() = sin(theta) * sin(phi);
                    point_eigen.z() = cos(theta);
                }
                else
                {
                    point_eigen.x() = sin(theta) * cos(phi);
                    point_eigen.y() = -sin(theta) * sin(phi);
                    point_eigen.z() = cos(theta);
                }
                point_eigen *= (double)depth_img.at<ushort>(i, j) / 1000.0;
                // Eigen::Vector3d w_point = it.second * point_eigen;
                Eigen::Vector3d w_point = point_eigen;
                // cout << "x = " << x << " ;y = " << y << " ;r = " << r << " ;theta = " << theta / M_PI * 180.0
                //      << " ;phi = " << phi / M_PI * 180.0 << " point_eigen.transpose() = " << point_eigen.transpose()
                //      << endl;

                PointT point;
                point.x = w_point.x();
                point.y = w_point.y();
                point.z = w_point.z();

                current->points.emplace_back(point);
            }
        }
        // depth filter and statistical removal
        // pcl::StatisticalOutlierRemoval<PointT> statistical_fileter;
        // statistical_fileter.setInputCloud(current);
        // statistical_fileter.setMeanK(50);
        // statistical_fileter.setStddevMulThresh(1.0);
        // statistical_fileter.filter(*current);
        *cloud_map += *current;
    }
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_map);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_map);

    pcl::visualization::PCLVisualizer viewer(
        "Matrix transformation example"); // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler(cloud_map, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(cloud_map, source_cloud_color_handler, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    cv::destroyAllWindows();
    // save cloud_map to pcd
    std::cout << "Saving to "
              << "/home/chrisliu/narwal_ws/src/CalculateImagesOverlapInHDMap/pointcloud/blender.pcd" << std::endl;
    pcl::io::savePCDFileASCII("/home/chrisliu/narwal_ws/src/CalculateImagesOverlapInHDMap/pointcloud/blender.pcd",
                              *cloud_map);
    return 0;
}