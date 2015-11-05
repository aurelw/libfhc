
#include "fasthullcollision.hpp"

#include <highgui.h>


namespace fhc {


FastHullCollision::FastHullCollision(const cv::Mat& camMat, int res_x, int res_y) :
    _rayCaster(camMat, res_x, res_y), _res_x(res_x), _res_y(res_y)
{
}


void FastHullCollision::setDistortionModel(float k1, float k2, float k3, 
        float p1, float p2)
{
    _rayCaster.setDistortionModel(k1, k2, k3, p1, p2);
}


void FastHullCollision::setGeometry(PointCloud::ConstPtr points, 
        const std::vector<pcl::Vertices>& indices)
{
    _rayCaster.setGeometry(points, indices);
}


void FastHullCollision::setGeometry(pcl::PolygonMesh::ConstPtr mesh) {
    _rayCaster.setGeometry(mesh);
}


void FastHullCollision::setup() {
    auto masks = _rayCaster.renderZMasks();
    _fgMask = std::get<0>(masks);
    _bgMask = std::get<1>(masks);

    cv::imwrite("crophull_fgmask.exr", _fgMask);
    cv::imwrite("crophull_bgmask.exr", _bgMask);

    _fgMaskMM = cv::Mat(_res_y, _res_x, CV_16UC1);
    _bgMaskMM = cv::Mat(_res_y, _res_x, CV_16UC1);
    _fgMask.convertTo(_fgMaskMM, CV_16UC1, 1000);
    _bgMask.convertTo(_bgMaskMM, CV_16UC1, 1000);
}


cv::Mat FastHullCollision::maskDepthImageMM(cv::Mat img) {
    /* check the input size */
    auto imgsize = img.size();
    if (imgsize.width != _res_x || imgsize.height != _res_y) {
        std::cout << "[ERROR] Wrong Image Size!!!" << std::endl;
        return cv::Mat::zeros(_res_y, _res_x, CV_16UC1);
    }

    cv::Mat out(_res_y, _res_x, CV_16UC1);

    for (int x=0; x<_res_x; x++) {
        for (int y=0; y<_res_y; y++) {
            int depth = img.at<uint16_t>(y,x);
            if (_fgMaskMM.at<uint16_t>(y,x) > depth) {
                out.at<uint16_t>(y,x) = 0;
                continue;
            }
            if (_bgMaskMM.at<uint16_t>(y,x) < depth) {
                out.at<uint16_t>(y,x) = 0;
                continue;
            }
            out.at<uint16_t>(y,x) = depth;
        }
    }

    return out;
}


cv::Mat FastHullCollision::maskDepthImage(cv::Mat img) {
    /* check the input size */
    auto imgsize = img.size();
    if (imgsize.width != _res_x || imgsize.height != _res_y) {
        std::cout << "[ERROR] Wrong Image Size!!!" << std::endl;
        return cv::Mat::zeros(_res_y, _res_x, CV_32FC1);
    }

    cv::Mat out(_res_y, _res_x, CV_32FC1);

    for (int x=0; x<_res_x; x++) {
        for (int y=0; y<_res_y; y++) {
            float depth = img.at<float>(y,x);
            if (_fgMask.at<float>(y,x) > depth) {
                out.at<float>(y,x) = 0;
                continue;
            }
            if (_bgMask.at<float>(y,x) < depth) {
                out.at<float>(y,x) = 0;
                continue;
            }
            out.at<float>(y,x) = depth;
        }
    }

    return out;
}

}

