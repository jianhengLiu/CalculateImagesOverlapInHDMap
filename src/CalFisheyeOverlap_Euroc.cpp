/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-07 14:38:02
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-06-02 11:24:33
 * @Description: Description
 */
#include <iostream>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
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
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define MAP_MAX_SIZE 10 // meter

camodocal::CameraPtr m_camera;
pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

float matchPointCloudsID(std::map<int, Eigen::Vector3d> &ids_points_img0,
                         std::map<int, Eigen::Vector3d> &ids_points_img1, PointCloudT &cloud_match,
                         cv::Mat &reproject_img0, cv::Mat &reproject_img1)
{
    reproject_img0 = cv::Mat::zeros(m_camera->imageHeight(), m_camera->imageWidth(), CV_8UC1);
    reproject_img1 = cv::Mat::zeros(m_camera->imageHeight(), m_camera->imageWidth(), CV_8UC1);
    for (auto &it0 : ids_points_img0)
    {
        auto it1 = ids_points_img1.find(it0.first);
        if (it1 != ids_points_img1.end())
        {
            cloud_match.points.emplace_back(it0.second(0), it0.second(1), it0.second(2));

            Eigen::Vector2d pixel;
            m_camera->spaceToPlane(it0.second, pixel);
            reproject_img0.at<uchar>((int)pixel(1), (int)pixel(0)) = it0.second(2) / MAP_MAX_SIZE * 255;
            m_camera->spaceToPlane(it1->second, pixel);
            reproject_img1.at<uchar>((int)pixel(1), (int)pixel(0)) = it1->second(2) / MAP_MAX_SIZE * 255;
        }
    }
    std::cout << "Match Numbers:" << cloud_match.points.size() << std::endl;
    int kernal_size = 10;
    cv::dilate(reproject_img0, reproject_img0,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernal_size, kernal_size)));
    cv::erode(reproject_img0, reproject_img0,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernal_size, kernal_size)));

    cv::dilate(reproject_img1, reproject_img1,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernal_size, kernal_size)));
    cv::erode(reproject_img1, reproject_img1,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernal_size, kernal_size)));

    int occupied_num0 = 0;
    int occupied_num1 = 0;
    int img_size = m_camera->imageHeight() * m_camera->imageWidth();
    for (int i = 0; i < img_size; i++)
    {
        if (reproject_img0.at<uchar>(i) != 0)
            occupied_num0++;
        if (reproject_img1.at<uchar>(i) != 0)
            occupied_num1++;
    }
    float overlap_ratio0 = (float)occupied_num0 / (float)img_size;
    float overlap_ratio1 = (float)occupied_num1 / (float)img_size;
    std::cout << "Occupied Numbers in Img0: " << occupied_num0 << std::endl;
    std::cout << "Occupied Ratio in Img0: " << overlap_ratio0 << std::endl;
    std::cout << "Occupied Numbers in Img1: " << occupied_num1 << std::endl;
    std::cout << "Occupied Ratio in Img1: " << overlap_ratio1 << std::endl;
    if (overlap_ratio0 > overlap_ratio1)
    {
        cout << "Overlap Ratio: " << overlap_ratio1 << endl;
    }
    else
    {
        cout << "Overlap Ratio: " << overlap_ratio0 << endl;
    }
    return overlap_ratio0;
}

void getPointCloudInFOV(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d &img_T_w,
                        std::map<int, Eigen::Vector3d> &ids_points_FOV, PointCloudT::Ptr cloud_FOV)
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

    cv::Mat depth_img(m_camera->imageHeight(), m_camera->imageWidth(), CV_16UC1, cv::Scalar(0xffff));
    cv::Mat ids_img(m_camera->imageHeight(), m_camera->imageWidth(), CV_32SC1, cv::Scalar(0xffffffff));
    for (int i = 0; i < transformed_cloud->points.size(); i++)
    {
        if (transformed_cloud->points[i].z > 0)
        {
            Eigen::Vector3d transformed_point(transformed_cloud->points[i].x, transformed_cloud->points[i].y,
                                              transformed_cloud->points[i].z);
            Eigen::Vector2d pixel;
            m_camera->spaceToPlane(transformed_point, pixel);

            if (pixel.x() >= 0 && pixel.x() < m_camera->imageWidth() && pixel.y() >= 0 &&
                pixel.y() < m_camera->imageHeight())
            {
                if (depth_img.at<ushort>((int)pixel(1), (int)pixel(0)) > transformed_point.norm() * 1000)
                {
                    if (ids_img.at<int>((int)pixel(1), (int)pixel(0)) != 0xffffffff)
                    {
                        ids_points_FOV.erase(ids_img.at<int>((int)pixel(1), (int)pixel(0)));
                    }
                    ids_img.at<int>((int)pixel(1), (int)pixel(0)) = i;
                    ids_points_FOV[i] = Eigen::Vector3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y,
                                                        transformed_cloud->points[i].z);

                    depth_img.at<ushort>((int)pixel(1), (int)pixel(0)) = transformed_point.norm() * 1000;
                }
            }
        }
    }

    // 由于点云密度低导致还是有很多被遮挡的点能够被加入,
    // 通过邻近深度比对进行过滤
    int kernal_size = 2;
    for (int x = kernal_size; x < m_camera->imageWidth() - kernal_size; x++)
    {
        for (int y = kernal_size; y < m_camera->imageHeight() - kernal_size; y++)
        {
            if (depth_img.at<int>(y, x) != 0xffff)
            {
                int k = 0;
                int error = 0;
                for (int i = -kernal_size; i < kernal_size; i++)
                {
                    for (int j = -kernal_size; j < kernal_size; j++)
                    {
                        if (x == 0 && j == 0)
                            continue;
                        if (depth_img.at<ushort>(y + j, x + i) != 0xffff)
                        {
                            if (depth_img.at<ushort>(y + j, x + i) < depth_img.at<ushort>(y, x))
                            {
                                error += depth_img.at<ushort>(y + j, x + i) - depth_img.at<ushort>(y, x);
                                k++;
                            }
                        }
                    }
                }
                if (k > 0)
                {
                    if (error / k < -50)
                        ids_points_FOV.erase(ids_img.at<int>(y, x));
                }
            }
        }
    }

    for (auto &it : ids_points_FOV)
    {
        cloud_FOV->points.emplace_back(cloud_in->points[it.first]);
    }
}

void TEST(PointCloudT::Ptr cloud_in, Eigen::Isometry3d w_T_img0, Eigen::Isometry3d w_T_img1, cv::Mat &reproject_img0,
          cv::Mat &reproject_img1)
{
    std::map<int, Eigen::Vector3d> ids_points_FOV_img0;
    PointCloudT::Ptr cloud_FOV_img0(new PointCloudT());
    getPointCloudInFOV(cloud_in, w_T_img0.inverse().matrix(), ids_points_FOV_img0, cloud_FOV_img0);

    std::map<int, Eigen::Vector3d> ids_points_FOV_img1;
    PointCloudT::Ptr cloud_FOV_img1(new PointCloudT());
    getPointCloudInFOV(cloud_in, w_T_img1.inverse().matrix(), ids_points_FOV_img1, cloud_FOV_img1);

    PointCloudT::Ptr cloud_match(new PointCloudT);
    float overlap_ratio =
        matchPointCloudsID(ids_points_FOV_img0, ids_points_FOV_img1, *cloud_match, reproject_img0, reproject_img1);
    pcl::transformPointCloud(*cloud_match, *cloud_match, w_T_img0.matrix());

    // filter out point cloud over 1m in z direction
    // pcl::PassThrough<PointT> pass;
    // pass.setInputCloud(cloud_in);
    // pass.setFilterFieldNa e("z");
    // pass.setFilterLimits(1.0, 10.0);
    // pass.filter(*cloud_in);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed1_cloud_color_handler(cloud_FOV_img0, 230, 20,
                                                                                              20); // Red
    viewer.removePointCloud("cloud_FOV_img0");
    viewer.addPointCloud(cloud_FOV_img0, transformed1_cloud_color_handler, "cloud_FOV_img0");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "cloud_FOV_img1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed2_cloud_color_handler(cloud_FOV_img1, 20, 230,
                                                                                              20); // Red
    viewer.removePointCloud("cloud_FOV_img1");
    viewer.addPointCloud(cloud_FOV_img1, transformed2_cloud_color_handler, "cloud_FOV_img1");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "cloud_FOV_img1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_cloud_color_handler(cloud_match, 20, 20,
                                                                                              230); // Red
    viewer.removePointCloud("match_cloud");
    viewer.addPointCloud(cloud_match, match_cloud_color_handler, "match_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "match_cloud");

    // return reproject_img;
    // cv::imshow("reproject_img", reproject_img);
}

int main(int argc, char **argv)
{
    std::string cfg_file;
    if (argv[1])
    {
        cfg_file = argv[1];
    }
    else
    {
        cfg_file = "/home/narwal/narwal_ws/src/CalculateImagesOverlapInHDMap/config/fisheye.yaml";
    }

    Config config = loadConfigFromYamlFile(cfg_file);

    std::map<long long, Eigen::Isometry3d> gt_data = loadGTData(config.gt_file);
    std::vector<long long> image_ids = loadImageIDs(config.image_path);

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cfg_file);

    // set transform from camera to body
    // Eigen::Isometry3d b_T_cam0 = Eigen::Isometry3d::Identity();
    // Eigen::Matrix3d b_R_cam0;
    // b_R_cam0 << 0.0148655429818, -0.999880929698, 0.00414029679422, 0.999557249008, 0.0149672133247, 0.025715529948,
    //     -0.0257744366974, 0.00375618835797, 0.999660727178;
    // b_T_cam0.rotate(b_R_cam0);
    // Eigen::Vector3d b_t_cam0;
    // b_t_cam0 << -0.0216401454975, -0.064676986768, 0.00981073058949;
    // b_T_cam0.translate(b_t_cam0);

    // Generate random number in given range
    srand((unsigned int)time(nullptr)); //初始化种子为随机值

    // read ply file and visualize
    PointCloudT::Ptr cloud_in(new PointCloudT);

    if (!config.ply_path.empty())
    {
        if (pcl::io::loadPLYFile(config.ply_path, *cloud_in) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file: %s\n", config.ply_path.c_str());
            return (-1);
        }
    }
    else if (!config.pcd_path.empty())
    {
        if (pcl::io::loadPCDFile(config.pcd_path, *cloud_in) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file: %s\n", config.pcd_path.c_str());
            return (-1);
        }
    }

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler(cloud_in, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud(cloud_in, source_cloud_color_handler, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "original_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setCameraPosition(0, 0, -20, 0, -1,
                             0); // Setting camera position: http://t.zoukankan.com/ghjnwk-p-10305796.html

    bool is_first = true;
    cv::namedWindow("image_pair", cv::WINDOW_AUTOSIZE);
    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        auto key = cv::waitKey(100);
        if ((key != 255 && key != -1) || is_first)
        {
            is_first = false;
            int img0_id = rand() % image_ids.size();
            int img1_id = rand() % image_ids.size();

            if (gt_data.find(image_ids[img0_id]) != gt_data.end() && gt_data.find(image_ids[img1_id]) != gt_data.end())
            {
                // w_T_img0 = w_T_b * b_T_cam
                auto w_T_img0 = gt_data[image_ids[img0_id]] * config.b_T_cam;
                auto w_T_img1 = gt_data[image_ids[img1_id]] * config.b_T_cam;

                cv::Mat reproject_img0, reproject_img1;
                TEST(cloud_in, w_T_img0, w_T_img1, reproject_img0, reproject_img1);

                if (!config.is_sim)
                {
                    cv::Mat image_pair;
                    cv::hconcat(cv::imread(config.image_path + "/" + std::to_string(image_ids[img0_id]) + ".png",
                                           cv::IMREAD_GRAYSCALE),
                                cv::imread(config.image_path + "/" + std::to_string(image_ids[img1_id]) + ".png",
                                           cv::IMREAD_GRAYSCALE),
                                image_pair);
                    cv::cvtColor(image_pair, image_pair, cv::COLOR_GRAY2BGR);
                    cv::applyColorMap(reproject_img0, reproject_img0, cv::COLORMAP_HOT);
                    cv::applyColorMap(reproject_img1, reproject_img1, cv::COLORMAP_HOT);
                    image_pair(cv::Rect(0, 0, reproject_img0.cols, reproject_img0.rows)) += reproject_img0;
                    image_pair(cv::Rect(reproject_img0.cols, 0, reproject_img1.cols, reproject_img1.rows)) +=
                        reproject_img1;
                    cv::putText(image_pair, "IMAGE0", cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 1,
                                cv::Scalar(0, 0, 255), 2);
                    cv::putText(image_pair, "IMAGE1", cv::Point(m_camera->imageWidth(), 30), cv::FONT_HERSHEY_COMPLEX,
                                1, cv::Scalar(0, 255, 0), 2);
                    cv::resize(image_pair, image_pair, cv::Size(1024, image_pair.rows * 1024 / image_pair.cols));
                    cv::imshow("image_pair", image_pair);
                }
                else
                {
                    cv::Mat image_pair;
                    cv::hconcat(reproject_img0, reproject_img1, image_pair);
                    cv::applyColorMap(image_pair, image_pair, cv::COLORMAP_HOT);
                    cv::putText(image_pair, "IMAGE0", cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 1,
                                cv::Scalar(0, 0, 255), 2);
                    cv::putText(image_pair, "IMAGE1", cv::Point(m_camera->imageWidth(), 30), cv::FONT_HERSHEY_COMPLEX,
                                1, cv::Scalar(0, 255, 0), 2);
                    cv::resize(image_pair, image_pair, cv::Size(1024, image_pair.rows * 1024 / image_pair.cols));
                    cv::imshow("image_pair", image_pair);
                }
            }
            else
            {
                is_first = true;
            }
        }
        viewer.spinOnce();
    }
    cv::destroyAllWindows();
    return (0);
}