
#include "simpleraycaster.hpp"

namespace fhc {

class FastHullCollision {

    public:

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    public:
    
        FastHullCollision(const cv::Mat& camMat, int res_x, int res_y);

        /*** wrapper for ray caster ***/
        /* Set the distortion model of the lens. Defaults to no distortion. */
        void setDistortionModel(float k1, float k2, float k3, float p1, float p2);
        /* Set a triangle mesh as the hit volume. */
        void setGeometry(PointCloud::ConstPtr points, 
                const std::vector<pcl::Vertices>& indices);
        /* Set a triangle mesh as the hit volume. */
        void setGeometry(pcl::PolygonMesh::ConstPtr mesh);

        /* Call to finalize setup (geometry,distortion,..) */
        void setup();

        cv::Mat maskDepthImageMM(const cv::Mat& img);
        cv::Mat maskDepthImage(const cv::Mat& img);
        int countInliers(const cv::Mat& img);
        int countInliersMM(const cv::Mat& img);



    private:

        SimpleRayCaster _rayCaster;

        int _res_x, _res_y;
        cv::Mat _bgMask, _fgMask;
        cv::Mat _bgMaskMM, _fgMaskMM;

        bool testImageSize(const cv::Mat& img);

};

}

