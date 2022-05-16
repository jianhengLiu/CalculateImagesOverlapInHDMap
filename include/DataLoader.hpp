/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-12 12:21:54
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-16 16:31:40
 * @Description: Description
 */
#pragma once
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

#include <opencv2/core/core.hpp>

struct Config
{
    bool is_sim;

    std::string gt_file;
    std::string image_path;
    std::string ply_path;
};

Config loadConfigFromYamlFile(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return {};
    }

    Config config;
    config.is_sim = static_cast<int>(fs["is_sim"]);
    fs["gt_file"] >> config.gt_file;
    fs["image_path"] >> config.image_path;
    fs["ply_path"] >> config.ply_path;

    std::cout << "is_sim: " << config.is_sim << std::endl;
    std::cout << "gt_file: " << config.gt_file << std::endl;
    std::cout << "image_path: " << config.image_path << std::endl;
    std::cout << "ply_path: " << config.ply_path << std::endl;

    return config;
}

std::vector<long long> loadImageIDs(std::string image_path)
{
    std::string image_file = image_path.substr(0, image_path.find_last_of('/')) + "/data.csv";

    std::vector<long long> image_ids;
    std::ifstream tr;
    tr.open(image_file.c_str());

    if (!tr.good())
    {
        return image_ids;
    }
    while (!tr.eof() && tr.good())
    {
        std::string line;
        char buf[1000];
        tr.getline(buf, 1000);

        long long id;
        if (sscanf(buf, "%lld", &id))
        {
            // std::cout << id << std::endl;
            image_ids.emplace_back(id);
        }
    }
    tr.close();
    return image_ids;
}

std::map<long long, Eigen::Isometry3d> loadGTData(std::string gtFile)
{
    std::string path = "/home/narwal/dataset/euroc/V2_01_easy";
    std::string defaultFile = path + "mav0/state_groundtruth_estimate0/data.csv";
    std::cout << "Loading gt data" << std::endl;

    if (gtFile.empty())
    {
        gtFile = defaultFile;
    }

    // std::map<long long, Eigen::Isometry3d> gt_data;
    std::map<long long, Eigen::Isometry3d> gt_data;
    std::ifstream tr;
    tr.open(gtFile.c_str());

    if (!tr.good())
    {
        return gt_data;
    }
    while (!tr.eof() && tr.good())
    {
        std::string line;
        char buf[1000];
        tr.getline(buf, 1000);

        long long id;
        double p1, p2, p3, qw, qx, qy, qz, v1, v2, v3, br1, br2, br3, bp1, bp2, bp3;
        if (17 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2,
                         &p3, &qw, &qx, &qy, &qz, &v1, &v2, &v3, &br1, &br2, &br3, &bp1, &bp2, &bp3))
        {
            // EuRoC format with bias GT.
            Eigen::Vector3d translation(p1, p2, p3);
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); //(quat, translation);
            pose.translate(translation);
            pose.rotate(quat);
            // Sophus::SE3 pose(quat, translation);
            // Eigen::Vector3d velocity(v1, v2, v3);
            // Eigen::Vector3d biasRot(br1, br2, br3);
            // Eigen::Vector3d biasPos(bp1, bp2, bp3);

            gt_data[id] = pose;

            // gtData[id] = dmvio::GTData(pose, velocity, biasRot, biasPos);
        }
        else if (8 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2, &p3, &qw, &qx, &qy, &qz))
        {
            // TUM-VI format
            Eigen::Vector3d translation(p1, p2, p3);
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            // Sophus::SE3 pose(quat, translation);
            Eigen::Vector3d velocity(0.0, 0.0, 0.0);
            Eigen::Vector3d biasRot(0.0, 0.0, 0.0);
            Eigen::Vector3d biasPos(0.0, 0.0, 0.0);

            // gtData[id] = dmvio::GTData(pose, velocity, biasRot, biasPos);
        }
    }
    tr.close();
    return gt_data;
}