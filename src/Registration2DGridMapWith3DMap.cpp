/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-07 14:38:02
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-11 17:32:13
 * @Description: Description
 */
#include "eigen3/Eigen/Core"
#include "pcl/common/io.h"
#include <iostream>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/console/time.h> // TicToc
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// extract point cloud at given height
void extractPointCloudAtGivenHeight(PointCloudT::Ptr cloud, PointCloudT::Ptr filtered_cloud, float height)
{
    // Create the filtering object
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(height - 0.01, height + 0.01);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*filtered_cloud);
}

Eigen::Matrix4f iterativeICP(PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_input, PointCloudT::Ptr cloud_output)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputTarget(cloud_target);
    icp.setTransformationEpsilon(1e-8); //设置前后两次迭代的转换矩阵的最大容差（epsilion），一旦两次迭代小于这个最大容差，则认为已经收敛到最优解，迭代停止。迭代停止条件之二，默认值为：0
    // icp.setMaxCorrespondenceDistance(0.01); //设置最大对应点的欧式距离，只有对应点之间的距离小于该设置值的对应点才作为ICP计算的点对。默认值为：1.7976931348623157e+308

    Eigen::Matrix4f t_T_s = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudT::Ptr prev_match_cloud;
    pcl::copyPointCloud(*cloud_input, *cloud_output);
    for (int i = 0; i < 30; ++i) {
        // PCL_INFO("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        prev_match_cloud = cloud_output;

        // Estimate
        icp.setInputSource(prev_match_cloud);
        icp.align(*cloud_output);

        // accumulate transformation between each Iteration
        t_T_s = icp.getFinalTransformation() * t_T_s;

        // if the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        if (fabs((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon()) {
            break;
            // icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.001);
        }

        // std::cout << "icp.getLastIncrementalTransformation()" << std::endl
        //           << icp.getLastIncrementalTransformation() << std::endl;
        // std::cout << "fabs((icp.getLastIncrementalTransformation() - prev).sum())" << std::endl
        //           << fabs((icp.getLastIncrementalTransformation() - prev).sum()) << std::endl;

        prev = icp.getLastIncrementalTransformation();

        // visualize current state
        // showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }
    return t_T_s.inverse();
}

int main(int argc, char** argv)
{
    // read ply file and visualize
    PointCloudT::Ptr cloud_in(new PointCloudT);

    std::string ply_path = "/home/narwal/narwal_ws/src/CalculateImagesOverlapInHDMap/pointcloud/data.ply";

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(ply_path, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file: %s\n", ply_path.c_str());
        return (-1);
    }
    std::cout << "Loaded file " << ply_path << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n"
              << std::endl;

    PointCloudT::Ptr target_grid_cloud(new PointCloudT);
    extractPointCloudAtGivenHeight(cloud_in, target_grid_cloud, 1.0);
    PointCloudT::Ptr input_grid_cloud(new PointCloudT);
    extractPointCloudAtGivenHeight(cloud_in, input_grid_cloud, 1.5);

    Eigen::Affine3f w_T_c1 = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    w_T_c1.translation() << 2.5, 0.0, 0.0;
    // The same rotation matrix as before; theta radians around Z axis
    float yaw = M_PI / 4.0;
    w_T_c1.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    w_T_c1.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()));
    std::cout << "w_T_c1:" << std::endl
              << w_T_c1.matrix() << std::endl;
    std::cout << "w_T_c1.inverse():" << std::endl
              << w_T_c1.inverse().matrix() << std::endl;

    PointCloudT::Ptr transformed_input_grid_cloud(new PointCloudT);
    // Executing the transformation
    pcl::transformPointCloud(*input_grid_cloud, *transformed_input_grid_cloud, w_T_c1.inverse());

    PointCloudT::Ptr match_cloud(new PointCloudT);

    time.tic();
    Eigen::Matrix4f w_T_i = iterativeICP(target_grid_cloud, transformed_input_grid_cloud, match_cloud);
    std::cout << "iterativeICP cost: " << time.toc() << " ms\n"
              << std::endl;
    std::cout << "w_T_i:" << std::endl
              << w_T_i << std::endl;

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
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed1_cloud_color_handler(target_grid_cloud, 230, 20, 20); // Red
    viewer.addPointCloud(target_grid_cloud, transformed1_cloud_color_handler, "grid_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed2_cloud_color_handler(transformed_input_grid_cloud, 20, 230, 20); // Red
    viewer.addPointCloud(transformed_input_grid_cloud, transformed2_cloud_color_handler, "transformed_grid_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_grid_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> input_grid_cloud_color_handler(input_grid_cloud, 255, 0, 255); // Red
    viewer.addPointCloud(input_grid_cloud, input_grid_cloud_color_handler, "input_grid_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_grid_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> match_cloud_color_handler(match_cloud, 20, 20, 230);
    viewer.addPointCloud(match_cloud, match_cloud_color_handler, "match_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "match_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPosition(800, 400); // Setting visualiser window position
    // Set camera position and orientation
    // viewer.setCameraPosition(0, 0, -10, 0, 0, -1, 0);
    while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }
    cv::destroyAllWindows();
    return (0);
}