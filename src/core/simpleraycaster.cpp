/* simpleraycaster.cpp -- Part of libfhc
 *
 * Copyright (C) 2016 CARGOMETER GmbH
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Author: Aurel Wildfellner
 */

#include "simpleraycaster.hpp"

#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>


namespace fhc {


SimpleRayCaster::SimpleRayCaster(const cv::Mat& camMat,
        int res_x, int res_y) : _res_x(res_x), _res_y(res_y), _camMat(camMat),
        _k1(0.0), _k2(0.0), _k3(0.0), _p1(0.0), _p2(0.0)
{
    cv::invert(camMat, _inverseCamMat);

    /* disect camera matrix */
    _fx = camMat.at<float>(0,0);
    _fy = camMat.at<float>(1,1);
    _cx = camMat.at<float>(0,2);
    _cy = camMat.at<float>(1,2);
};


void SimpleRayCaster::setDistortionModel(float k1, float k2, float k3, 
        float p1, float p2) 
{
    _k1 = k1;
    _k2 = k2;
    _k3 = k3;
    _p1 = p1;
    _p2 = p2;
}
    


void SimpleRayCaster::setGeometry(
        PointCloud::ConstPtr points, const std::vector<pcl::Vertices>& indices) 
{
    _fPoints = points;
    _fIndices = indices;
}


void SimpleRayCaster::setGeometry(pcl::PolygonMesh::ConstPtr mesh,
        const Eigen::Affine3f& transform) 
{
    PointCloud::Ptr tmp(new PointCloud);
    pcl::fromPCLPointCloud2<pcl::PointXYZ>(mesh->cloud, *tmp);
    pcl::transformPointCloud<pcl::PointXYZ, float>(*tmp, *tmp, transform);
    _fPoints = tmp;
    _fIndices = mesh->polygons;
 }


void SimpleRayCaster::printGeometry() {
    std::cout << "fx: " << _fx << "   fy: " << _fy << std::endl;
    std::cout << "cx: " << _cx << "   cy: " << _cy << std::endl;
    for (auto vertices : _fIndices) {
        auto pa = _fPoints->at(vertices.vertices[0]);
        auto pb = _fPoints->at(vertices.vertices[1]);
        auto pc = _fPoints->at(vertices.vertices[2]);
        std::cout << "Triangle: " << 
            "(" << pa.x << ","
                << pa.y << ","
                << pa.z << ") - "
            "(" << pb.x << ","
                << pb.y << ","
                << pb.z << ") - "
            "(" << pc.x << ","
                << pc.y << ","
                << pc.z << ")" << std::endl;
    }
}


void SimpleRayCaster::renderDebugImage(int width, int height) {

    for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
            Eigen::Vector2f co((float)x/(float)width,(float)y/(float)height);
            auto zvalues = castRay(co);
            if (zvalues.size() > 0) {
                std::cout << "0";
            } else {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }

    }


std::tuple<cv::Mat, cv::Mat> SimpleRayCaster::renderZMasks() {
    cv::Mat fgMask(_res_y, _res_x, CV_32F);
    cv::Mat bgMask(_res_y, _res_x, CV_32F);

    for (int y=0; y<_res_y; y++) {
        for (int x=0; x<_res_x; x++) {

            Eigen::Vector2f co((float)x/(float)_res_x,(float)y/(float)_res_y);
            auto zvalues = castRay(co);
            float fgPixel = 0.0f;
            float bgPixel = 0.0f;

            /* only valid masks if there were 2 intersections */
            if (zvalues.size() == 2) {
                if (zvalues[0] < zvalues[1]) {
                    fgPixel = zvalues[0];
                    bgPixel = zvalues[1];
                } else {
                    fgPixel = zvalues[1];
                    bgPixel = zvalues[0];
                }
            }

            fgMask.at<float>(y,x) = fgPixel;
            bgMask.at<float>(y,x) = bgPixel;
        }
    } 

    return std::make_tuple(fgMask, bgMask);
}


std::vector<float> SimpleRayCaster::castRay(Eigen::Vector2f &p) {

    std::vector<float> zvalues;

    /* distort the point with the lens parameters */
    Eigen::Vector2f pDist = distortPoint(p);

    /* convert 2d coordinate (0.0-1.0) to proper image coordinate */
    Eigen::Vector3f aP(pDist[0]*_res_x, 
                       pDist[1]*_res_y, 
                       1.0); 

    /* project with inverse camera calibration matrix 
     * to 3d vector in camera space */
    cv::Mat ptMat = (cv::Mat_<float>(3,1) << aP[0], aP[1], aP[2]);
    cv::Mat vec3d = _inverseCamMat * ptMat; 
    Eigen::Vector3f pvec(vec3d.at<float>(0,0),
                         vec3d.at<float>(1,0),
                         vec3d.at<float>(2,0));

    std::vector<Eigen::Vector3f> isecs = intersectRay(pvec);
    for (auto isec : isecs) {
        zvalues.push_back(isec.norm());
    }

    return zvalues;
}


std::vector<Eigen::Vector3f> SimpleRayCaster::intersectRay(Eigen::Vector3f vec) {
    std::vector<Eigen::Vector3f> isecs;

    /* iterate over all triangles */
    for (auto vertices : _fIndices) {
        auto pa = _fPoints->at(vertices.vertices[0]).getVector3fMap();
        auto pb = _fPoints->at(vertices.vertices[1]).getVector3fMap();
        auto pc = _fPoints->at(vertices.vertices[2]).getVector3fMap();
        
        Eigen::Vector3f isec;
        if (intersectRayTriangle(vec, pa, pb, pc, isec)) {
            isecs.push_back(isec);
        }
    }

    return isecs;
}

bool SimpleRayCaster::intersectRayTriangle(const Eigen::Vector3f ray,
        const Eigen::Vector3f a, const Eigen::Vector3f b, const Eigen::Vector3f c,
        Eigen::Vector3f& isec)
{

    /* As discribed by:
     * http://geomalgorithms.com/a06-_intersect-2.html#intersect_RayTriangle%28%29 */

    const Eigen::Vector3f p(0,0,0);
    const Eigen::Vector3f u = b - a;
    const Eigen::Vector3f v = c - a;
    const Eigen::Vector3f n = u.cross(v);
    const float n_dot_ray = n.dot(ray);

    if (std::fabs(n_dot_ray) < 1e-9) {
        return false;
    }


    const float r = n.dot(a-p) / n_dot_ray;

    if (r < 0) {
        return false;
    }

    // the ray intersection point
    isec = ray * r;

    const Eigen::Vector3f w = p + r * ray - a;
    const float denominator = u.dot(v) * u.dot(v) - u.dot(u) * v.dot(v);
    const float s_numerator = u.dot(v) * w.dot(v) - v.dot(v) * w.dot(u);
    const float s = s_numerator / denominator;
    if (s < 0 || s > 1) {
        return false;
    }

    const float t_numerator = u.dot(v) * w.dot(u) - u.dot(u) * w.dot(v);
    const float t = t_numerator / denominator;
    if (t < 0 || s+t > 1) {
        return false;
    }

    return true;
}


Eigen::Vector2f SimpleRayCaster::distortPoint(Eigen::Vector2f &p) {
    // convert from (0.0 - 1.0) coordinates to "-0._ - 0._" and c centered.
    float xu = ((p[0]*_res_x) - _cx) / (float)_fx;
    float yu = ((p[1]*_res_y) - _cy) / (float)_fy;

    float r2 = xu*xu + yu*yu; 

    /* radial distortion */
    float xd = xu * (1 + _k1*r2 + _k2*r2*r2 + _k3*r2*r2*r2);
    float yd = yu * (1 + _k1*r2 + _k2*r2*r2 + _k3*r2*r2*r2);

    /* tangential distortion */
    xd = xd + (2 * _p1 * xu * yu + _p2 * (r2 + 2 * xu * xu));
    yd = yd + (_p1 * (r2 + 2 * yu * yu) + 2 * _p2 * xu * yu);

    // convert back to "0.0 - 1.0"
    xd = ((xd * _fx) + _cx) / (float)_res_x;
    yd = ((yd * _fy) + _cy) / (float)_res_y;

    Eigen::Vector2f pd;
    pd << xd, yd;
    return pd;
}

}

