/* utils.cpp -- Part of libfhc
 *
 * Copyright (C) 2016 CARGOMETER GmbH
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Author: Aurel Wildfellner
 */

#include "utils.hpp"

#include <string>

namespace fhc {

cv::Mat convertToPNG(const cv::Mat &input) {
    cv::Mat output(input.rows, input.cols, CV_8U);
    input.convertTo(output, CV_8U, 255);
    cv::normalize(input, output, 0, 255, cv::NORM_MINMAX);
    return output;
}

std::string getOpenCVImageType(int type)
{
    std::map<int, std::string> mappings = {
        {CV_8U, "CV_8U"},
        {CV_8UC1, "CV_8UC1"},
        {CV_8UC2, "CV_8UC2"},
        {CV_8UC3, "CV_8UC3"},
        {CV_8UC4, "CV_8UC4"},
        {CV_8S, "CV_8S"},
        {CV_8SC1, "CV_8SC1"},
        {CV_8SC2, "CV_8SC2"},
        {CV_8SC3, "CV_8SC3"},
        {CV_8SC4, "CV_8SC4"},
        {CV_16U, "CV_16U"},
        {CV_16UC1, "CV_16UC1"},
        {CV_16UC2, "CV_16UC2"},
        {CV_16UC3, "CV_16UC3"},
        {CV_16UC4, "CV_16UC4"},
        {CV_16S, "CV_16S"},
        {CV_16SC1, "CV_16SC1"},
        {CV_16SC2, "CV_16SC2"},
        {CV_16SC3, "CV_16SC3"},
        {CV_16SC4, "CV_16SC4"},
        {CV_32S, "CV_32S"},
        {CV_32SC1, "CV_32SC1"},
        {CV_32SC2, "CV_32SC2"},
        {CV_32SC3, "CV_32SC3"},
        {CV_32SC4, "CV_32SC4"},
        {CV_32F, "CV_32F"},
        {CV_32FC1, "CV_32FC1"},
        {CV_32FC2, "CV_32FC2"},
        {CV_32FC3, "CV_32FC3"},
        {CV_32FC4, "CV_32FC4"},
        {CV_64F, "CV_64F"},
        {CV_64FC1, "CV_64FC1"},
        {CV_64FC2, "CV_64FC2"},
        {CV_64FC3, "CV_64FC3"},
        {CV_64FC4, "CV_64FC4"}
    };

    if (mappings.count(type)) {
        return mappings[type];
    } else {
        return "NONE_TYPE";
    }
}

}

