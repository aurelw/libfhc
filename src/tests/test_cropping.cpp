#include <highgui.h>

#include <pcl/conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

#include "fasthullcollision.hpp"
#include "utils.hpp"

#define WITH_EXTRINSIC 0


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace fhc;

int main(int argc, char** argv) {

    int res_x = 512;
    int res_y = 424;
    float fx = 367.06039;
    float fy = 367.06039;
    float cx = res_x / 2;
    float cy = res_y / 2;
    cv::Mat camMat = (cv::Mat_<float>(3,3)<< fx, 0.0, cx,
                                             0.0, fy, cy,
                                             0.0, 0.0, 1.0);

    /* load the depth image from an exr and convert */
    
#if ! WITH_EXTRINSIC
    cv::Mat inImage = cv::imread("crop_hull_depthimage.exr", CV_LOAD_IMAGE_ANYDEPTH);
    std::cout << "Input image type: " << getOpenCVImageType(inImage.type()) << std::endl;
    cv::Mat depthImage(res_y, res_x, CV_32FC1);
    int from_to[] = {0,0};
    cv::mixChannels(&inImage, 1, &depthImage, 1, from_to, 1);
    cv::Mat mmRangeImage(res_y, res_x, CV_16UC1);
    depthImage.convertTo(mmRangeImage, CV_16UC1, 1000);
#else
    cv::Mat mmRangeImage = cv::imread("crop_hull_depthimage.png", CV_LOAD_IMAGE_ANYDEPTH);
    std::cout << "Input image type: " << getOpenCVImageType(mmRangeImage.type()) << std::endl;
#endif

    /* load geometry */
#if WITH_EXTRINSIC
    Eigen::Affine3f camextr = pcl::getTransformation(-1.7217004, 0, 3.14074, 
             pcl::deg2rad(131.22336 - 90), pcl::deg2rad(-179.64928), pcl::deg2rad(129.01939)).inverse();
#endif

    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadPolygonFileSTL("crop_hull.stl", *mesh);

    FastHullCollision fastHC(camMat, res_x, res_y);
#if WITH_EXTRINSIC
    fastHC.setGeometry(mesh, camextr);
#else
    fastHC.setGeometry(mesh);
#endif
    fastHC.setup();

#if ! WITH_EXTRINSIC
    cv::Mat outImage = fastHC.maskDepthImage(depthImage);
    cv::imwrite("masked_depth_image.exr", outImage);
#endif

    std::cout << "Mask size: " << fastHC.getMaskSize() << std::endl;

    cv::Mat outImageMM = fastHC.maskDepthImageMM(mmRangeImage);
    cv::imwrite("masked_depth_image_mm.exr", outImageMM);

    cv::imwrite("debug_input_out_mm.exr", mmRangeImage);
}


