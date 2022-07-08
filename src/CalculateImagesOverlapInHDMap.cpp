/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-07 14:38:02
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-11 17:33:27
 * @Description: Description
 */
#include <iostream>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>  // for PointCloud
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
// #camera calibration
// model_type: PINHOLE
// camera_name: camera
// image_width: 640
// image_height: 480

// distortion_parameters:
//    k1: 0
//    k2: 0
//    p1: 0
//    p2: 0
// projection_parameters:
//    fx: 380
//    fy: 380
//    cx: 320
//    cy: 240

void matchPointCloudsID(std::map<int, Eigen::Vector3d> &points_1,
                        std::map<int, Eigen::Vector3d> &points_2, PointCloudT::Ptr match_cloud,
                        cv::Mat &reproject_img)
{
    Eigen::Matrix3d camera_matrix;
    camera_matrix << 380, 0, 320, 0, 380, 240, 0, 0, 1;
    for (auto &it : points_1)
    {
        if (points_2.find(it.first) != points_2.end())
        {
            match_cloud->points.push_back(pcl::PointXYZ(it.second(0), it.second(1), it.second(2)));

            Eigen::Vector3d pixel = camera_matrix * it.second / it.second.z();
            reproject_img.at<uchar>((int)pixel(1), (int)pixel(0)) = 255;
        }
    }
    std::cout << "Match Numbers:" << match_cloud->points.size() << std::endl;
    cv::dilate(reproject_img, reproject_img,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));
    cv::erode(reproject_img, reproject_img,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));

    int occupied_num = 0;
    int img_size     = 480 * 640;
    for (int i = 0; i < img_size; i++)
    {
        if (reproject_img.at<uchar>(i) == 255)
            occupied_num++;
    }
    std::cout << "Occupied Numbers:" << occupied_num << std::endl;
    std::cout << "Occupied Ratio:" << (float)occupied_num / img_size << std::endl;
}

void getPointCloudWithCameraModel(PointCloudT::Ptr                cloud,
                                  std::map<int, Eigen::Vector3d> &points_FOV,
                                  PointCloudT::Ptr                filtered_cloud)
{
    Eigen::Matrix3d camera_matrix;
    camera_matrix << 380, 0, 320, 0, 380, 240, 0, 0, 1;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        // if (cloud->points[i].intensity > 0.5)
        {
            Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            Eigen::Vector3d pixel = camera_matrix * point / point.z();
            if (pixel.x() >= 0 && pixel.x() < 640 && pixel.y() >= 0 && pixel.y() < 480 &&
                pixel.z() > 0)
            {
                points_FOV[i] = point;
                filtered_cloud->points.push_back(cloud->points[i]);
            }
        }
    }
}

int main(int argc, char **argv)
{
    // read ply file and visualize
    PointCloudT::Ptr cloud(new PointCloudT);

    std::string ply_path = "/home/narwal/narwal_ws/src/CalculateImagesOverlapInHDMap/data/data.ply";
    if (pcl::io::loadPLYFile(ply_path, *cloud) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file: %s\n", ply_path.c_str());
        return (-1);
    }

    Eigen::Affine3f w_transform_c1 = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    w_transform_c1.translation() << 2.5, 0.0, 0.0;
    // The same rotation matrix as before; theta radians around Z axis
    float yaw = M_PI / 4.0;
    w_transform_c1.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    std::cout << w_transform_c1.matrix() << std::endl;
    std::cout << w_transform_c1.inverse().matrix() << std::endl;

    // Executing the transformation
    PointCloudT::Ptr transformed_cloud(new PointCloudT());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud, *transformed_cloud, w_transform_c1.inverse());

    std::map<int, Eigen::Vector3d> points_FOV_1;
    PointCloudT::Ptr               cloud_camera1(new PointCloudT());
    getPointCloudWithCameraModel(cloud, points_FOV_1, cloud_camera1);

    std::map<int, Eigen::Vector3d> points_FOV_2;
    PointCloudT::Ptr               cloud_camera2(new PointCloudT());
    getPointCloudWithCameraModel(transformed_cloud, points_FOV_2, cloud_camera2);
    // for visualization
    pcl::transformPointCloud(*cloud_camera2, *cloud_camera2, w_transform_c1);

    PointCloudT::Ptr match_cloud(new PointCloudT);
    cv::Mat          reproject_img = cv::Mat::zeros(480, 640, CV_8UC1);
    matchPointCloudsID(points_FOV_1, points_FOV_2, match_cloud, reproject_img);

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // filter out point cloud over 1m in z direction
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 10.0);
    pass.filter(*cloud);

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler(cloud, 255,
                                                                                        255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                            "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed1_cloud_color_handler(
        transformed_cloud, 230, 20, 20);  // Red
    viewer.addPointCloud(cloud_camera1, transformed1_cloud_color_handler, "cloud_camera1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                            "cloud_camera1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed2_cloud_color_handler(
        transformed_cloud, 20, 230, 20);  // Red
    viewer.addPointCloud(cloud_camera2, transformed2_cloud_color_handler, "cloud_camera2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                            "cloud_camera2");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_cloud_color_handler(
        match_cloud, 20, 20, 230);  // Red
    viewer.addPointCloud(match_cloud, match_cloud_color_handler, "match_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                            "match_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);  // Setting background to a dark grey
    // viewer.setPosition(800, 400); // Setting visualiser window position
    cv::namedWindow("reproject_img", cv::WINDOW_AUTOSIZE);
    while (!viewer.wasStopped())
    {  // Display the visualiser until 'q' key is pressed
        cv::imshow("reproject_img", reproject_img);
        cv::waitKey(1);
        viewer.spinOnce();
    }
    cv::destroyAllWindows();
    return (0);
}