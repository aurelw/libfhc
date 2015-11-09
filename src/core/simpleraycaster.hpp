
#include <pcl/common/common_headers.h>
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>

#include <cv.h>


namespace fhc {


class SimpleRayCaster {

    public:

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    public:

        SimpleRayCaster(const cv::Mat& camMat, int res_x, int res_y);

        /* Set an optional distortion model for the camera lens.
         * Defaults to no distortion. */
        void setDistortionModel(float k1, float k2, float k3, float p1, float p2);
        /* Set the mesh as a series of points and indices for (only) triangles. */
        void setGeometry(PointCloud::ConstPtr points, 
                const std::vector<pcl::Vertices>& indices);
        /* Set the geometry as a pcl mesh object. */
        void setGeometry(pcl::PolygonMesh::ConstPtr mesh);

        /* Returns a floating point foreground/background mask
         * in a tuple. For empty volumes the mask is set to 0.0 */
        std::tuple<cv::Mat, cv::Mat> renderZMasks();

        /* Renders an ASCII art debug image. */
        void renderDebugImage(int width=32, int height=26);
        /* Prints al vertices and triangles for debugging. */
        void printGeometry();


    protected:

        /* Casts a ray at a given image coordinate. (x,y) is between 0.0 and 1.0
         * and starts at the upper left corner.
         * Returns a list of z values, one of each intersection. */
        std::vector<float> castRay(Eigen::Vector2f &p);
        /* Intersec a ray starting at 0,0,0 and in direction of the parameter vec.
         * Returns a list of 3D intersection points. */
        std::vector<Eigen::Vector3f> intersectRay(Eigen::Vector3f vec);
        bool intersectRayTriangle(const Eigen::Vector3f ray, 
                const Eigen::Vector3f a, 
                const Eigen::Vector3f b, 
                const Eigen::Vector3f c,
                Eigen::Vector3f& isec);

        /* Distort a point in the image with coordinates 0.0 - 1.0 on
         * the x axis and 0.0 - 1.0 on the y axis starting at the top
         * left corner */
        Eigen::Vector2f distortPoint(Eigen::Vector2f &p);

    private:

        const int _res_x, _res_y;
        cv::Mat _camMat;
        cv::Mat _inverseCamMat;

        float _fx, _fy, _cx, _cy;
        float _k1, _k2, _k3, _p1, _p2;

        PointCloud::ConstPtr _fPoints;
        std::vector<pcl::Vertices> _fIndices;

};

}
