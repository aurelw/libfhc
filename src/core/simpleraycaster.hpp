
#include <pcl/common/common_headers.h>
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>

#include <cv.h>



class SimpleRayCaster {

    public:

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    public:

        SimpleRayCaster(const cv::Mat& camMat, int res_x, int res_y);

        void setGeometry(PointCloud::ConstPtr points, 
                const std::vector<pcl::Vertices>& inidices);
        void setGeometry(pcl::PolygonMesh::ConstPtr mesh);
        void printGeometry();
        void renderDebugImage(int width=32, int height=26);

        std::tuple<cv::Mat, cv::Mat> renderZMasks();

    protected:

        /* Casts a ray at a given image coordinate.
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

        Eigen::Vector2f distortPoint(Eigen::Vector2f &p);

    private:

        const int _res_x, _res_y;
        cv::Mat _camMat;
        cv::Mat _inverseCamMat;

        float _k1, _k2, _k3, _p1, _p2;

        PointCloud::ConstPtr _fPoints;
        std::vector<pcl::Vertices> _fIndices;

};

