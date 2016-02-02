# libfhc
libfhc is a small utility library for collision tests with depth data or range images. It is built with the objective of speed and performance while providing a very simple, high level, single class interface. The basic functionality is to count the number of points within a given 3D volume or to mask the input data with this volume.

### Principle
The input depth data must be a range or depth image (also known as an organized pointcloud). Usually this is the raw image provided by a depth camera like a kinect or ToF cameras. The 3D *hit volume* is specified by an input mesh. The mesh is raycasted into a *foreground* and *background mask* image in a way, that is perfectly aligned with the input data to test on. For each pixel in the input range image, the depth is tested to be within the interval given by the foreground and background mask. If so, it is an inlier, otherwise not. This way, na image can be masked or inliers can be counted by a simple and very performant series of arithmetic operations.

Ray-casting the initial masks and therefore initializing the library is also slow and should only be performed on small meshes. It is not the objective of optimization. The library is built for a setup with static masks and a very high number of collision tests. Feel free to add a more performant raycaster tho ;)

### Volume Properties
Given by the underlying principle of the library, the hit volume should be a *convex hull*. However, as long as a ray, starting from a camera pixel, does not intersect the mesh more than two times (which would always be the case with a convex hull) the method still works. Therefore, convexity is not a hard requirement. Future versions may include *collision masks* for several levels and support convexity to some degree. The library also handles defects and non-watertight meshes, though of course it will break the measurement in some pixels.

### Example
The entire interface is provided by the class FastHullCollision.
```C++
// Camera Matrix - This is the model of the depth camera.
int res_x = 512;
int res_y = 424;
float fx = 367.06039;
float fy = 367.06039;
float cx = res_x / 2;
float cy = res_y / 2;
cv::Mat camMat = (cv::Mat_<float>(3,3)<< fx, 0.0, cx,
                                         0.0, fy, cy,
                                         0.0, 0.0, 1.0);

// The Hit Volume Mesh
pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
pcl::io::loadPolygonFileSTL("crop_hull.stl", *mesh);

// Load Input Data
cv::Mat inImage = cv::imread("crop_hull_depthimage.exr", CV_32FC1);

/* Setting up libfhc */
FastHullCollision fastHC(camMat, res_x, res_y);
fastHC.setGeometry(mesh);
fastHC.setup();

// Collision Tests - Mask the Image
cv::Mat outImage = fastHC.maskDepthImage(inImage);
// or just count Inliers.
int num_inliers = fastHC.countInliers(inImage);
```

### Camera Calibration and Model
One key motivation for this library and for rewritting a simple raycaster was to include a distortion model directly when generating rays. If not set, the library defaults to no distortion and just uses the bare camera matrix.
```C++
/* Setting up FHC with a camera distortion model */
FastHullCollision fastHC(camMat, res_x, res_y);
fastHC.setGeometry(mesh);
fastHC.setDistortionModel(0.091808669, -0.27344111, 0.094382137, 0.0, 0.0);
fastHC.setup();
```

### Input Image Format
Images are handled by OpenCV's cv::Mat class. The pixel type can be either floating point or 16 bit integer (CV_32FC1, CV_16UC1). 16 bit, unsigned integers is a very common format for depth cameras. In libfhc, one *tick* of the 16 bit value corresponds to 1/1000th Unit of the 3D mesh. This was chosen because the input data libfhc was initialy designed for encoded 1 mm to each increment of the 16 bit value. If you happen to have a different mapping, scale the hit volume geometry or convert the input image to floating point pixels.

### Compile and Install
Building and installation is done via cmake:

```bash
git clone https://github.com/aurelw/libfhc.git
cd libfhc
mkdir build
cd build
cmake ..
make
make install # as root
```

OpenCV, Eigen as well as libpcl are required. libpcl is needed to handle the input meshes and for some transformations, which is a rather heavy weight dependency in that manner and may be removed later on. At the moment the library was used in build environments which rely on pcl anyways, so feel free to request such a change if you need it.

### Typical Use Cases
  * We use this library for building triggers on depth video streams which start further processing.
  * Hit boxes as buttons on a glass window, see [CraftUI](https://github.com/aurelw/craftui).
  * Many hitboxes to classify moving patterns in scenes.
