#ifndef POINTCLOUDTRIANGULATION_H
#define POINTCLOUDTRIANGULATION_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

#include <map>
#include <string>
#include "pclalgorithm.h"

//#define PI 3.141592653589793238462643383279
#define DEFAULT_SEARCH_RADIUS 0.025
#define DEFAULT_MULTIPLIER 2.5
#define DEFAULT_MAX_NEAREST_NEIGHBORS 100
#define DEFAULT_MAX_SURFACE_ANGLE M_PI/4 //0.785398163397448  pi/4
#define DEFAULT_MIN_ANGLE M_PI/18  //10°
#define DEFAULT_MAX_ANGLE 2*M_PI/3 //120°
#define DEFAULT_NORMAL_CONSISTENCY true

class PointCloudTriangulation
{
private:
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    std::map<std::string, PCLFilter* > algorithms;
    
public:
    PointCloudTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr);
    PointCloudTriangulation(std::string);
    PointCloudTriangulation();

    void addFilter(std::string, PCLFilter*);
    void applyFilter(std::string);
    void loadCloudFromFile(std::string);
    void surfaceSmoothing();
    void reconstruct();
    void saveTriangulation(std::string);
    PCLFilter* getFilter(std::string);
    pcl::PointCloud<pcl::PointNormal>::Ptr getCloud();
    pcl::PolygonMesh getTriangulation();    
};

#endif // POINTCLOUDTRIANGULATION_H
