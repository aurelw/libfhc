#include <highgui.h>

#include <pcl/conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

#include "simpleraycaster.hpp"
#include "utils.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace fhc;


int main(int argc, char** argv) {

    /* test camera */
    int res_x = 512;
    int res_y = 424;
    float fx = 367.06039;
    float fy = 367.06039;
    float cx = 261.33521;
    float cy = 207.7928;
    /*
    float cx = res_x / 2;
    float cy = res_y / 2;
    */
    cv::Mat camMat = (cv::Mat_<float>(3,3)<< fx, 0.0, cx,
                                             0.0, fy, cy,
                                             0.0, 0.0, 1.0);
    /*
    Eigen::Affine3f camextr = pcl::getTransformation(-1.7217004, 0, 3.14074, 
             pcl::deg2rad(131.22336 - 90), pcl::deg2rad(-179.64928), pcl::deg2rad(129.01939)).inverse();
    */

    /* load geometry */
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadPolygonFileSTL("geometry.stl", *mesh);

    PointCloud::Ptr geometryCloud(new PointCloud);
    pcl::fromPCLPointCloud2<pcl::PointXYZ>(mesh->cloud, *geometryCloud);

    SimpleRayCaster rc(camMat, res_x, res_y);
    rc.setDistortionModel(0.091808669, -0.27344111, 0.094382137, 0.0, 0.0);
    //rc.setGeometry(geometryCloud, mesh->polygons);
    //rc.setGeometry(mesh, camextr);
    rc.setGeometry(mesh);
    //rc.printGeometry();

    rc.renderDebugImage(80,40);

    auto masks = rc.renderZMasks();
    auto fgMask = std::get<0>(masks);
    auto bgMask = std::get<1>(masks);
    cv::imwrite("fgmask.png", convertToPNG(fgMask));
    cv::imwrite("bgmask.png", convertToPNG(bgMask));

}

