#ifndef PCLSTATISTICALOUTLIERREMOVAL_H
#define PCLSTATISTICALOUTLIERREMOVAL_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcfilter.h"

//number of point to use for mean distance estimation
#define PARAM_MEAN_K 0 //"mean k"
#define PARAM_STD_DEV_MUL_TH 1 //"standard deviation multipilier threashold"

// default values
#define DEFAULT_MEAN_K 50
#define DEFAULT_STD_DEV_MUL_TH 0.1

class PCStatisticalOutlierRemoval : public PointCloudFilter
{
public:

    PCStatisticalOutlierRemoval();
    void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setParameter(int, double);
    double getParameter(int);

private:
    std::map<int, double> parameters;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

};

#endif // PCLSTATISTICALOUTLIERREMOVAL_H
