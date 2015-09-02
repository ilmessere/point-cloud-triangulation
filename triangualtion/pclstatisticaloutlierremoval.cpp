#include "pclstatisticaloutlierremoval.h"

PCLStatisticalOutlierRemoval::PCLStatisticalOutlierRemoval()
{
    parameters[PARAM_MEAN_K] = (double) DEFAULT_MEAN_K;
    parameters[PARAM_STD_DEV_MUL_TH] = (double) DEFAULT_STD_DEV_MUL_TH;
}

void PCLStatisticalOutlierRemoval::apply(pcl::PointCloud<pcl::PointNormal>::Ptr cloud){
    std::cout<<"Number of points before the Statistical Outlier Removal: "<<cloud->width*cloud->height<<std::endl;
    sor.setInputCloud (cloud);
    sor.setMeanK ((int)parameters[PARAM_MEAN_K]);
    sor.setStddevMulThresh ((float)parameters[PARAM_STD_DEV_MUL_TH]);
    sor.filter (*cloud);
    std::cout<<"Number of points after the Statistical Outlier Removal: "<<cloud->width*cloud->height<<std::endl;
}

void PCLStatisticalOutlierRemoval::setParameter(int param_code, double value){
    parameters[param_code] = value;
}

double PCLStatisticalOutlierRemoval::getParameter(int param_code){
    return parameters[param_code];
}

