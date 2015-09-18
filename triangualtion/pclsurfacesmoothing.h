#ifndef PCLSURFACESMOOTHING_H
#define PCLSURFACESMOOTHING_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <map>

#include "pclalgorithm.h"

#define DEFAULT_SEARCH_RADIUS 0.03
#define DEFAULT_POLYNOMIAL_FIT true
#define DEFAULT_COMPUTE_NORMALS false

#define PARAM_SEARCH_RADIUS 0
#define PARAM_POLYNOMIAL_FIT 1
#define PARAM_COMPUTE_NORMALS 2

class PCLSurfaceSmoothing : public PCLFilter
{
public:
    PCLSurfaceSmoothing();
    void apply(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
    void setParameter(int, double);
    double getParameter(int);

private:
    std::map<int, double> parameters;
    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
};

#endif // PCLSURFACESMOOTHING_H
