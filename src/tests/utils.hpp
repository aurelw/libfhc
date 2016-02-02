/* utils.hpp -- Part of libfhc
 *
 * Copyright (C) 2016 CARGOMETER GmbH
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Author: Aurel Wildfellner
 */

#include <cv.h>

namespace fhc {

/* convert an image to 8bit grayscale with minmax normalization */
cv::Mat convertToPNG(const cv::Mat &input);

/* returns for e.g. "CV_32F" for CV_32F. */
std::string getOpenCVImageType(int type);

}

