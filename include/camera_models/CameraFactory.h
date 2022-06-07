/*
 * @Author: Jianheng Liu
 * @Date: 2022-05-12 11:00:53
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-05-12 11:08:58
 * @Description: Description
 */
#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camera_models/Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
