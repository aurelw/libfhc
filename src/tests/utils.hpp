
#include <cv.h>

namespace fhc {

/* convert an image to 8bit grayscale with minmax normalization */
cv::Mat convertToPNG(const cv::Mat &input);

/* returns for e.g. "CV_32F" for CV_32F. */
std::string getOpenCVImageType(int type);

}

