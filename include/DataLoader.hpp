/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-12 12:21:54
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-12 15:42:52
 * @Description: Description
 */
#pragma once
#include <fstream>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

std::vector<Eigen::Isometry3d> loadGTData(std::string gtFile)
{
    std::string path = "/home/narwal/dataset/euroc/V2_01_easy";
    std::string defaultFile = path + "mav0/state_groundtruth_estimate0/data.csv";
    std::cout << "Loading gt data" << std::endl;

    if (gtFile.empty()) {
        gtFile = defaultFile;
    }

    std::vector<Eigen::Isometry3d> gt_data;
    std::ifstream tr;
    tr.open(gtFile.c_str());

    if (!tr.good()) {
        return gt_data;
    }
    while (!tr.eof() && tr.good()) {
        std::string line;
        char buf[1000];
        tr.getline(buf, 1000);

        long long id;
        double p1, p2, p3, qw, qx, qy, qz, v1, v2, v3, br1, br2, br3, bp1, bp2, bp3;
        if (17 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2, &p3, &qw, &qx, &qy, &qz, &v1, &v2, &v3, &br1, &br2, &br3, &bp1, &bp2, &bp3)) {
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

            gt_data.emplace_back(pose);

            // gtData[id] = dmvio::GTData(pose, velocity, biasRot, biasPos);

        } else if (8 == sscanf(buf, "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &id, &p1, &p2, &p3, &qw, &qx, &qy, &qz)) {
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