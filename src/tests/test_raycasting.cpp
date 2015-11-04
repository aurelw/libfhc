#include <highgui.h>

#include <pcl/conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

#include "simpleraycaster.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace fhc;


cv::Mat convertToPNG(const cv::Mat& input) {
    cv::Mat output(input.rows, input.cols, CV_8U);
    input.convertTo(output, CV_8U, 255);
    cv::normalize(input, output, 0, 255, cv::NORM_MINMAX);
    return output;
}


int main(int argc, char** argv) {

    /* test camera */
    int res_x = 512;
    int res_y = 424;
    float fx = 367.06039;
    float fy = 367.06039;
    float cx = 261.33521;
    float cy = 207.7928;
    cv::Mat camMat = (cv::Mat_<float>(3,3)<< fx, 0.0, cx,
                                             0.0, fy, cy,
                                             0.0, 0.0, 1.0);

    /* load geometry */
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadPolygonFileSTL("geometry.stl", *mesh);

    PointCloud::Ptr geometryCloud(new PointCloud);
    pcl::fromPCLPointCloud2<pcl::PointXYZ>(mesh->cloud, *geometryCloud);

    SimpleRayCaster rc(camMat, res_x, res_y);
    rc.setDistortionModel(0.091808669, -0.27344111, 0.094382137, 0.0, 0.0);
    //rc.setGeometry(geometryCloud, mesh->polygons);
    rc.setGeometry(mesh);
    //rc.printGeometry();

    rc.renderDebugImage(80,40);

    auto masks = rc.renderZMasks();
    auto fgMask = std::get<0>(masks);
    auto bgMask = std::get<1>(masks);
    cv::imwrite("fgmask.png", convertToPNG(fgMask));
    cv::imwrite("bgmask.png", convertToPNG(bgMask));

}

