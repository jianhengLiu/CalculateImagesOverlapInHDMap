/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-07 14:38:02
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-12 17:02:43
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
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <camera_models/CameraFactory.h>
#include <camera_models/PinholeCamera.h>

#include "DataLoader.hpp"

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

camodocal::CameraPtr m_camera;

void matchPointCloudsID(std::map<int, Eigen::Vector3d>& ids_points_img0, std::map<int, Eigen::Vector3d>& ids_points_img1, PointCloudT::Ptr cloud_match, cv::Mat& reproject_img)
{
    for (auto& it : ids_points_img0) {
        if (ids_points_img1.find(it.first) != ids_points_img1.end()) {
            cloud_match->points.emplace_back(it.second(0), it.second(1), it.second(2));

            Eigen::Vector2d pixel;
            m_camera->spaceToPlane(it.second, pixel);
            reproject_img.at<uchar>((int)pixel(1), (int)pixel(0)) = 255;
        }
    }
    std::cout << "Match Numbers:" << cloud_match->points.size() << std::endl;
    cv::dilate(reproject_img, reproject_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));
    cv::erode(reproject_img, reproject_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));

    int occupied_num = 0;
    int img_size = m_camera->imageHeight() * m_camera->imageWidth();
    for (int i = 0; i < img_size; i++) {
        if (reproject_img.at<uchar>(i) == 255)
            occupied_num++;
    }
    std::cout << "Occupied Numbers:" << occupied_num << std::endl;
    std::cout << "Occupied Ratio:" << (float)occupied_num / img_size << std::endl;
}

void getPointCloudInFOV(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d& img_T_w, std::map<int, Eigen::Vector3d>& ids_FOV, PointCloudT::Ptr cloud_FOV)
{
    // Executing the transformation
    PointCloudT::Ptr transformed_cloud(new PointCloudT());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, img_T_w);

    // 这里不能用filter，因为filter会对点云进行排序，导致点云的id发生变化
    // In camera coordinate, camera faces z axis direction (z > 0)
    // pcl::PassThrough<PointT> pass;
    // pass.setInputCloud(transformed_cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, FLT_MAX); // FLT_MAX is the max value of float
    // pass.filter(*transformed_cloud);

    for (int i = 0; i < transformed_cloud->points.size(); i++) {
        if (transformed_cloud->points[i].z > 0) {
            Eigen::Vector3d point(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z);
            Eigen::Vector2d pixel;
            m_camera->spaceToPlane(point, pixel);

            // Eigen::Vector3d pixel = camera_matrix * point / point.z();
            if (pixel.x() >= 0 && pixel.x() < m_camera->imageWidth() && pixel.y() >= 0 && pixel.y() < m_camera->imageHeight()) {
                ids_FOV[i] = point;
                cloud_FOV->points.push_back(cloud_in->points[i]);
            }
        }
    }
}

void test()
{
    // Load the camera model
}

int main(int argc, char** argv)
{
    std::vector<Eigen::Isometry3d> gt_data = loadGTData("/home/narwal/dataset/euroc/V2_01_easy/mav0/state_groundtruth_estimate0/data.csv");
    Eigen::Isometry3d b_T_cam0 = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d b_R_cam0;
    b_R_cam0 << 0.0148655429818, -0.999880929698, 0.00414029679422,
        0.999557249008, 0.0149672133247, 0.025715529948,
        -0.0257744366974, 0.00375618835797, 0.999660727178;
    b_T_cam0.rotate(b_R_cam0);
    Eigen::Vector3d b_t_cam0;
    b_t_cam0 << -0.0216401454975, -0.064676986768, 0.00981073058949;
    b_T_cam0.translate(b_t_cam0);

    // Generate random number in given range
    srand((unsigned int)time(nullptr)); //初始化种子为随机值
    int img0_id = rand() % gt_data.size();
    int img1_id = rand() % gt_data.size();

    auto w_T_img0 = gt_data[img0_id] * b_T_cam0;
    auto w_T_img1 = gt_data[img1_id] * b_T_cam0;

    m_camera
        = camodocal::CameraFactory::instance()->generateCameraFromYamlFile("/home/narwal/narwal_ws/src/CalculateImagesOverlapInHDMap/config/euroc/euroc_config.yaml");

    // read ply file and visualize
    PointCloudT::Ptr cloud_in(new PointCloudT);

    std::string ply_path = "/home/narwal/narwal_ws/src/CalculateImagesOverlapInHDMap/pointcloud/data.ply";
    if (pcl::io::loadPLYFile(ply_path, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file: %s\n", ply_path.c_str());
        return (-1);
    }

    std::map<int, Eigen::Vector3d> ids_points_FOV_img0;
    PointCloudT::Ptr cloud_FOV_img0(new PointCloudT());
    getPointCloudInFOV(cloud_in, w_T_img0.inverse().matrix(), ids_points_FOV_img0, cloud_FOV_img0);
    // pcl::transformPointCloud(*cloud_FOV_img0, *cloud_FOV_img0, w_T_img0.matrix());

    std::map<int, Eigen::Vector3d> ids_points_FOV_img1;
    PointCloudT::Ptr cloud_FOV_img1(new PointCloudT());
    getPointCloudInFOV(cloud_in, w_T_img1.inverse().matrix(), ids_points_FOV_img1, cloud_FOV_img1);
    // pcl::transformPointCloud(*cloud_FOV_img1, *cloud_FOV_img1, w_T_img1.matrix());

    PointCloudT::Ptr cloud_match(new PointCloudT);
    cv::Mat reproject_img = cv::Mat::zeros(m_camera->imageHeight(), m_camera->imageWidth(), CV_8UC1);
    matchPointCloudsID(ids_points_FOV_img0, ids_points_FOV_img1, cloud_match, reproject_img);
    pcl::transformPointCloud(*cloud_match, *cloud_match, w_T_img0.matrix());

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // filter out point cloud over 1m in z direction
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 10.0);
    pass.filter(*cloud_in);

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler(cloud_in, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(cloud_in, source_cloud_color_handler, "original_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed1_cloud_color_handler(cloud_FOV_img0, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_FOV_img0, transformed1_cloud_color_handler, "cloud_FOV_img0");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_FOV_img0");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed2_cloud_color_handler(cloud_FOV_img1, 20, 230, 20); // Red
    viewer.addPointCloud(cloud_FOV_img1, transformed2_cloud_color_handler, "cloud_FOV_img1");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_FOV_img1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_cloud_color_handler(cloud_match, 20, 20, 230); // Red
    viewer.addPointCloud(cloud_match, match_cloud_color_handler, "match_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "match_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPosition(800, 400); // Setting visualiser window position
    viewer.setCameraPosition(0, 0, -20, 0, -1, 0); // Setting camera position: http://t.zoukankan.com/ghjnwk-p-10305796.html

    while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
        cv::imshow("reproject_img", reproject_img);
        cv::waitKey(1);
        // std::cout << "viewer.getViewerPose() = " << std::endl
        //           << viewer.getViewerPose().matrix()
        //           << std::endl;
        viewer.spinOnce();
    }
    cv::destroyAllWindows();
    return (0);
}