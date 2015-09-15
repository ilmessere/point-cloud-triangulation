#ifndef PCLVOXELGRIDFILTER_H
#define PCLVOXELGRIDFILTER_H

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <map>
#include "pcfilter.h"

#define DEFAULT_LEAF_SIZE 0.01f
#define PARAM_LEAF_SIZE_X 0 //"leaf size x"
#define PARAM_LEAF_SIZE_Y 1 //"leaf size y"
#define PARAM_LEAF_SIZE_Z 2 //"leaf size z"

class PCVoxelGridFilter : public PointCloudFilter
{
public:
    PCVoxelGridFilter();
    void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    //void setParameter(int, std::string);
    void setParameter(int, double);
    double getParameter(int);
    bool isValidParam(int);
private:
    pcl::VoxelGrid< pcl::PointXYZ> voxel_grid_filter;
    std::map<int, double> params;
};

#endif // PCLVOXELGRIDFILTER_H
