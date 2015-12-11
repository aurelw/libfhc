
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
        /* Set a triangle mesh as the hit volume. Should be a convex 
         * hull or at least a "sufficiently convex", watertight mesh 
         * as viewed from the origin. A subsequent call overrides the
         * geometry. */
        void setGeometry(PointCloud::ConstPtr points, 
                const std::vector<pcl::Vertices>& indices);
        /* Set a triangle mesh as the hit volume, alternative interface. */
        void setGeometry(pcl::PolygonMesh::ConstPtr mesh, Eigen::Affine3f transform=Eigen::Affine3f::Identity());

        /* Call to finalize setup (geometry,distortion,..) */
        void setup();

        /* Mask a u16 bit integer depth image at a scale 1:1000
         * (1000 in depth image is 1.0 in geometry hull and 
         * computed masks). Every point which is not within
         * the volume is set to 0. */
        cv::Mat maskDepthImageMM(const cv::Mat& img);
        /* Mas a floating point depth image. Points which are
         * not within the volume are set to 0.0 */
        cv::Mat maskDepthImage(const cv::Mat& img);
        /* Count the number of points within the volume. 
         * Input <img> is a floating point image. */
        int countInliers(const cv::Mat& img);
        /* Count the inliers in a unsigned 16 bit integer
         * image at a scale of 1:1000 */
        int countInliersMM(const cv::Mat& img);

    private:

        cv::Rect calculateROI();
        bool emptyRow(int y);
        bool emptyCol(int x);

        SimpleRayCaster _rayCaster;

        int _res_x, _res_y;
        cv::Mat _bgMask, _fgMask;
        cv::Mat _bgMaskMM, _fgMaskMM;
        cv::Rect _roi;

        bool testImageSize(const cv::Mat& img);

};

}

