/* fasthullcollision.cpp -- Part of libfhc
 *
 * Copyright (C) 2016 CARGOMETER GmbH
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Author: Aurel Wildfellner
 */

#include "fasthullcollision.hpp"

#include <assert.h>
#include <highgui.h>


namespace fhc {


FastHullCollision::FastHullCollision(const cv::Mat& camMat, int res_x, int res_y) :
    _rayCaster(camMat, res_x, res_y), _res_x(res_x), _res_y(res_y),
    _roi(0, 0, res_x, res_y)
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


void FastHullCollision::setGeometry(pcl::PolygonMesh::ConstPtr mesh, Eigen::Affine3f transform) {
    _rayCaster.setGeometry(mesh, transform);
}


void FastHullCollision::setup() {
    auto masks = _rayCaster.renderZMasks();
    _fgMask = std::get<0>(masks);
    _bgMask = std::get<1>(masks);

    _fgMaskMM = cv::Mat(_res_y, _res_x, CV_16UC1);
    _bgMaskMM = cv::Mat(_res_y, _res_x, CV_16UC1);
    _fgMask.convertTo(_fgMaskMM, CV_16UC1, 1000);
    _bgMask.convertTo(_bgMaskMM, CV_16UC1, 1000);

    _roi = calculateROI();

    _maskSize = computeMaskSize();
}


cv::Mat FastHullCollision::maskDepthImageMM(const cv::Mat& img) {
    /* check the input size */
    assert(testImageSize(img));

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


cv::Mat FastHullCollision::maskDepthImage(const cv::Mat& img) {
    /* check the input size */
    assert(testImageSize(img));

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


int FastHullCollision::countInliers(const cv::Mat& img) {
    /* check the input size */
    assert(testImageSize(img));

    int inliers = 0;

    for (int x=_roi.x; x < (_roi.x + _roi.width); x++) {
        for (int y=_roi.y; y < (_roi.y + _roi.height); y++) {
            float depth = img.at<float>(y,x);
            if (_fgMask.at<float>(y,x) < depth &&
                _bgMask.at<float>(y,x) > depth)
            {   
                inliers++;
            }
        }
    }

    return inliers;
}


int FastHullCollision::countInliersMM(const cv::Mat& img) {
    /* check the input size */
    assert(testImageSize(img));

    int inliers = 0;

    for (int x=_roi.x; x < (_roi.x + _roi.width); x++) {
        for (int y=_roi.y; y < (_roi.y + _roi.height); y++) {
            float depth = img.at<uint16_t>(y,x);
            if (_fgMaskMM.at<uint16_t>(y,x) < depth &&
                _bgMaskMM.at<uint16_t>(y,x) > depth)
            {   
                inliers++;
            }
        }
    }

    return inliers;
}


bool FastHullCollision::testImageSize(const cv::Mat& img) {
    auto imgsize = img.size();
    return (imgsize.width != _res_x || imgsize.height != _res_y);
}


bool FastHullCollision::emptyCol(int x) {
    for (int y=0; y<_res_y; y++) {
        if (_fgMaskMM.at<uint16_t>(y,x) != 0) return false;
        if (_bgMaskMM.at<uint16_t>(y,x) != 0) return false;
    }

    return true;
}


bool FastHullCollision::emptyRow(int y) {
    for (int x=0; x<_res_x; x++) {
        if (_fgMaskMM.at<uint16_t>(y,x) != 0) return false;
        if (_bgMaskMM.at<uint16_t>(y,x) != 0) return false;
    }

    return true;
}


cv::Rect FastHullCollision::calculateROI() {
    int x0 = 0;
    int x1 = 0;
    int y0 = 0;
    int y1 = 0;

    for (int x=0; x<_res_x; x++) {
        if (emptyCol(x)) {
            x0 = x+1;
        } else {
            break;
        }
    }

    for (int x=x0; x<_res_x; x++) {
        if (!emptyCol(x)) {
            x1 = x;
        }
    }

    for (int y=0; y<_res_y; y++) {
        if (emptyRow(y)) {
            y0 = y+1;
        } else {
            break;
        }
    }

    for (int y=y0; y<_res_y; y++) {
        if (!emptyCol(y)) {
            y1 = y;
        }
    }

    /* an empty mask has no valid roi */ 
    if (x0 > x1) {
        x0 = x1 = 0;
    }
    if (y0 > y1) {
        y0 = y1 = 0;
    }

    return cv::Rect(x0, y0, x1-x0, y1-y0);

}


cv::Mat FastHullCollision::getBgMask() {
    return _bgMask.clone();
}


cv::Mat FastHullCollision::getFgMask() {
    return _fgMask.clone();
}


cv::Mat FastHullCollision::getBgMaskMM() {
    return _bgMaskMM.clone();
}


cv::Mat FastHullCollision::getFgMaskMM() {
    return _fgMaskMM.clone();
}


int FastHullCollision::getMaskSize() {
    return _maskSize;
}


int FastHullCollision::computeMaskSize() {
    int masksize = 0;
    for (int x=_roi.x; x < (_roi.x + _roi.width); x++) {
        for (int y=_roi.y; y < (_roi.y + _roi.height); y++) {
            if (_fgMask.at<float>(y,x) != 0) masksize++;
        }
    }
    return masksize;
}


}


