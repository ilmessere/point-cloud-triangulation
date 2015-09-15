#ifndef PCLALGORITHMS_H
#define PCLALGORITHMS_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <string>
#include <vector>

/*
 * pure virtual class used as an interface
*/

class PointCloudFilter
{
public:
    virtual void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) = 0;
    virtual void setParameter(int, double) = 0;
    virtual double getParameter(int) = 0;
};

#endif // PCLALGORITHMS_H
